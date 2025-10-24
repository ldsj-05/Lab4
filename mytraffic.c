#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>          
#include <linux/interrupt.h>     
#include <linux/kthread.h>      
#include <linux/delay.h>          
#include <linux/jiffies.h>       
#include <linux/mutex.h>          
#include <linux/fs.h>             
#include <linux/uaccess.h>        
#include <linux/device.h>

#define DEVICE_NAME "mytraffic"
#define MAJOR_NUM 61         
#define CLASS_NAME "traffic"

static int red_gpio    = 67;      // example from lab sysfs demo
static int yellow_gpio = 68;      // reset to the real pin ***change***
static int green_gpio  = 69;      // resest to real pin  ***change ***
static int btn_mode_gpio = 26;    // BTN0 mode 
static int btn_ped_gpio  = 27;    // pedestrian
module_param(red_gpio, int, 0444);
module_param(yellow_gpio, int, 0444);
module_param(green_gpio, int, 0444);
module_param(btn_mode_gpio, int, 0444);
module_param(btn_ped_gpio, int, 0444);
MODULE_PARM_DESC(red_gpio, "GPIO number for RED LED");
MODULE_PARM_DESC(yellow_gpio, "GPIO number for YELLOW LED");
MODULE_PARM_DESC(green_gpio, "GPIO number for GREEN LED");
MODULE_PARM_DESC(btn_mode_gpio, "GPIO number for BTN0");
MODULE_PARM_DESC(btn_ped_gpio, "GPIO number for BTN1");

enum mode_t { MODE_NORMAL = 0, MODE_FLASH_RED, MODE_FLASH_YELLOW };
static enum mode_t mode = MODE_NORMAL;     // start in normal
static int cycle_rate_hz = 1;              // Hz (1 cycle = 1s)
static bool ped_waiting = false;         
static struct mutex state_lock;           
// ----- thread + irq bookkeeping -----
static struct task_struct *traffic_task;
static bool running = true;                // thread loop flag
static int irq_mode = -1;                  // IRQ number for BTN0
static int irq_ped  = -1;                  // IRQ number for BTN1
static const unsigned long debounce_ms = 200;
static unsigned long last_mode_jiffies;    // debounce for BTN0
static unsigned long last_ped_jiffies;     // debounce for BTN1


static int btn_mode_level = 1;             // high
static int btn_ped_level  = 1;

static inline int period_ms(void)
{
    int hz;
    mutex_lock(&state_lock);
    hz = cycle_rate_hz;
    mutex_unlock(&state_lock);
    if (hz < 1) hz = 1;
    return 1000 / hz;
}

static inline void led_set(int gpio, int on)
{
    if (gpio_is_valid(gpio))
        gpio_set_value(gpio, on ? 1 : 0);
}

static inline void all_off(void)
{
    led_set(red_gpio, 0);
    led_set(yellow_gpio, 0);
    led_set(green_gpio, 0);
}

static inline void show_green(void)  { led_set(green_gpio, 1); led_set(yellow_gpio, 0); led_set(red_gpio, 0); }
static inline void show_yellow(void) { led_set(green_gpio, 0); led_set(yellow_gpio, 1); led_set(red_gpio, 0); }
static inline void show_red(void)    { led_set(green_gpio, 0); led_set(yellow_gpio, 0); led_set(red_gpio, 1); }
static inline void show_red_yellow(void) { led_set(green_gpio, 0); led_set(yellow_gpio, 1); led_set(red_gpio, 1); }

static irqreturn_t btn_mode_isr(int irq, void *dev_id)
{
    unsigned long now = jiffies;
    if (time_before(now, last_mode_jiffies + msecs_to_jiffies(debounce_ms)))
        return IRQ_HANDLED;
    last_mode_jiffies = now;

    btn_mode_level = gpio_get_value(btn_mode_gpio);
    if (btn_mode_level == 0) { // pressed (active-low)
        mutex_lock(&state_lock);
        // cycle modes: normal, flash-red, flash-yellow, normal
        mode = (mode == MODE_FLASH_YELLOW) ? MODE_NORMAL : (mode + 1);
        mutex_unlock(&state_lock);
    }
    return IRQ_HANDLED;
}

static irqreturn_t btn_ped_isr(int irq, void *dev_id)
{
    unsigned long now = jiffies;
    if (time_before(now, last_ped_jiffies + msecs_to_jiffies(debounce_ms)))
        return IRQ_HANDLED;
    last_ped_jiffies = now;

    btn_ped_level = gpio_get_value(btn_ped_gpio);
    if (btn_ped_level == 0) { // pressed 
        mutex_lock(&state_lock);
        if (mode == MODE_NORMAL)
            ped_waiting = true;  // apply at next stop phase
        mutex_unlock(&state_lock);
    }
    return IRQ_HANDLED;
}

// traffic loop
static int traffic_thread(void *arg)
{
    while (!kthread_should_stop() && running) {
        int pm = period_ms();

        // read current button levels
        btn_mode_level = gpio_get_value(btn_mode_gpio);
        btn_ped_level  = gpio_get_value(btn_ped_gpio);

        // Extra credit
        if (btn_mode_level == 0 && btn_ped_level == 0) {
            led_set(red_gpio, 1);
            led_set(yellow_gpio, 1);
            led_set(green_gpio, 1);

            // stay here while both are held
            while (!kthread_should_stop() && running) {
                if (gpio_get_value(btn_mode_gpio) != 0 || gpio_get_value(btn_ped_gpio) != 0)
                    break; 
                msleep(30);
            }

            // reset to initial state
            mutex_lock(&state_lock);
            mode = MODE_NORMAL;
            cycle_rate_hz = 1;
            ped_waiting = false;
            mutex_unlock(&state_lock);
            all_off();
            msleep(150);
            continue;
        }

        // do one phase based on current mode
        mutex_lock(&state_lock);
        switch (mode) {
        case MODE_NORMAL:
            // green 3 cycles
            show_green();
            mutex_unlock(&state_lock);
            msleep(pm * 3);

            // yellow 1 cycle
            mutex_lock(&state_lock);
            show_yellow();
            mutex_unlock(&state_lock);
            msleep(pm * 1);

            // stop phase
            mutex_lock(&state_lock);
            if (ped_waiting) {
                show_red_yellow();            // red+yellow for 5 cycles
                ped_waiting = false;          // consume request
                mutex_unlock(&state_lock);
                msleep(pm * 5);
            } else {
                show_red();                   // red 2 cycles
                mutex_unlock(&state_lock);
                msleep(pm * 2);
            }
            break;

        case MODE_FLASH_RED:
            show_red();
            mutex_unlock(&state_lock);
            msleep(pm);
            led_set(red_gpio, 0);
            msleep(pm);
            break;

        case MODE_FLASH_YELLOW:
            show_yellow();
            mutex_unlock(&state_lock);
            msleep(pm);
            led_set(yellow_gpio, 0);
            msleep(pm);
            break;
        }
    }

    all_off();
    return 0;
}

static ssize_t my_read(struct file *f, char __user *ubuf, size_t len, loff_t *off)
{
    char kbuf[256];
    int n, r, y, g, rate;
    enum mode_t m;
    bool ped;

    if (*off) return 0; // one-shot read

    mutex_lock(&state_lock);
    m    = mode;
    rate = cycle_rate_hz;
    ped  = ped_waiting;
    r = gpio_get_value(red_gpio);
    y = gpio_get_value(yellow_gpio);
    g = gpio_get_value(green_gpio);
    mutex_unlock(&state_lock);

    n = scnprintf(kbuf, sizeof(kbuf),
        "mode=%s\nrate=%d Hz\nstatus=red %s, yellow %s, green %s\npedestrian=%s\n",
        (m==MODE_NORMAL)?"normal":(m==MODE_FLASH_RED)?"flashing-red":"flashing-yellow",
        rate,
        r?"on":"off", y?"on":"off", g?"on":"off",
        ped?"present":"absent");

    if (copy_to_user(ubuf, kbuf, n)) return -EFAULT;
    *off = n;
    return n;
}

static ssize_t my_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
    char kbuf[16];
    int n = (len < (sizeof(kbuf)-1)) ? len : (int)sizeof(kbuf)-1;
    int v;

    if (copy_from_user(kbuf, ubuf, n)) return -EFAULT;
    kbuf[n] = '\0';

    if (kstrtoint(kbuf, 10, &v) == 0 && v >= 1 && v <= 9) {
        mutex_lock(&state_lock);
        cycle_rate_hz = v;
        mutex_unlock(&state_lock);
    }
    return len;
}

static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .read  = my_read,
    .write = my_write,
};

// gpio helpers 
static int setup_gpio(int gpio, const char *name, bool output, int initval)
{
    int ret;

    if (!gpio_is_valid(gpio)) {
        pr_err("mytraffic: %s: invalid GPIO %d\n", name, gpio);
        return -EINVAL;
    }
    ret = gpio_request(gpio, name);
    if (ret) {
        pr_err("mytraffic: %s: gpio_request(%d) failed: %d\n", name, gpio, ret);
        return ret;
    }

    if (output) ret = gpio_direction_output(gpio, initval ? 1 : 0);
    else        ret = gpio_direction_input(gpio);

    if (ret) {
        pr_err("mytraffic: %s: direction failed: %d\n", name, ret);
        gpio_free(gpio);
        return ret;
    }
    return 0;
}

static void free_gpios(void)
{
    if (gpio_is_valid(red_gpio))       gpio_free(red_gpio);
    if (gpio_is_valid(yellow_gpio))    gpio_free(yellow_gpio);
    if (gpio_is_valid(green_gpio))     gpio_free(green_gpio);
    if (gpio_is_valid(btn_mode_gpio))  gpio_free(btn_mode_gpio);
    if (gpio_is_valid(btn_ped_gpio))   gpio_free(btn_ped_gpio);
}

static int __init my_init(void)
{
    int ret;

    mutex_init(&state_lock);

    // LEDs start OFF buttons are inputs 
    if ((ret = setup_gpio(red_gpio,      "led_red",    true,  0))) goto err_gpio;
    if ((ret = setup_gpio(yellow_gpio,   "led_yellow", true,  0))) goto err_gpio;
    if ((ret = setup_gpio(green_gpio,    "led_green",  true,  0))) goto err_gpio;
    if ((ret = setup_gpio(btn_mode_gpio, "btn_mode",   false, 0))) goto err_gpio;
    if ((ret = setup_gpio(btn_ped_gpio,  "btn_ped",    false, 0))) goto err_gpio;

    // IRQs use falling + rising 
    ret = request_irq(gpio_to_irq(btn_mode_gpio), btn_mode_isr,
            IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                "mytraffic_btn_mode", NULL);
    if (ret) { pr_err("mytraffic: request_irq(mode) failed: %d\n", ret); goto err_gpio; }
    irq_mode = gpio_to_irq(btn_mode_gpio);

    ret = request_irq(gpio_to_irq(btn_ped_gpio), btn_ped_isr,
                      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                      "mytraffic_btn_ped", NULL);
    if (ret) { pr_err("mytraffic: request_irq(ped) failed: %d\n", ret); goto err_irq_mode; }
    irq_ped = gpio_to_irq(btn_ped_gpio);

    // char device
    ret = register_chrdev(MAJOR_NUM, DEVICE_NAME, &fops);
    if (ret < 0) { pr_err("mytraffic: register_chrdev failed: %d\n", ret); goto err_irq_both; }
    {
        static struct class *cls;
        static struct device *dev;

        cls = class_create(THIS_MODULE, CLASS_NAME);
        if (IS_ERR(cls)) { ret = PTR_ERR(cls); pr_err("mytraffic: class_create failed\n"); goto err_chrdev; }

        dev = device_create(cls, NULL, MKDEV(MAJOR_NUM, 0), NULL, DEVICE_NAME);
        if (IS_ERR(dev)) { ret = PTR_ERR(dev); pr_err("mytraffic: device_create failed\n"); class_destroy(cls); goto err_chrdev; }

    }

    // start the timing thread
    running = true;
    traffic_task = kthread_run(traffic_thread, NULL, "mytraffic_thread");
    if (IS_ERR(traffic_task)) {
        ret = PTR_ERR(traffic_task);
        pr_err("mytraffic: kthread_run failed: %d\n", ret);
        goto err_chrdev;
    }

    pr_info("mytraffic: loaded (R=%d Y=%d G=%d | BTN0=%d BTN1=%d) — /dev/%s (major=%d)\n",
            red_gpio, yellow_gpio, green_gpio, btn_mode_gpio, btn_ped_gpio,
            DEVICE_NAME, MAJOR_NUM);
    return 0;

err_chrdev:
    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
err_irq_both:
    if (irq_ped  >= 0) free_irq(irq_ped,  NULL);
err_irq_mode:
    if (irq_mode >= 0) free_irq(irq_mode, NULL);
err_gpio:
    all_off();
    free_gpios();
    return ret;
}

static void __exit my_exit(void)
{
    running = false;
    if (traffic_task && !IS_ERR(traffic_task))
        kthread_stop(traffic_task);

    if (irq_mode >= 0) free_irq(irq_mode, NULL);
    if (irq_ped  >= 0) free_irq(irq_ped,  NULL);

    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);

    all_off();
    free_gpios();

    pr_info("mytraffic: unloaded\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yash Patel pately@bu.edu Leah Jones ldsj@bu.edu");
MODULE_DESCRIPTION("EC535 Lab 4 — Traffic Light Controller");
MODULE_VERSION("1.0");

module_init(my_init);
module_exit(my_exit);