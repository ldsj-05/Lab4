// file: pately_ldsj_lab4/mytraffic.c
// EC535 Lab 4 - Traffic Light Controller
// Authors: Yash Patel (pately@bu.edu) and Leah Jones (ldsj@bu.edu)

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
static int btn_ped_gpio  = 46;    // pedestrian
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
//static struct mutex state_lock;     

#include <linux/spinlock.h>   /* add this at top with the other includes */
static spinlock_t state_lock;
static struct class *my_class;
static struct device *my_dev;

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

/**
 * period_ms() - Calculate cycle period in milliseconds from current Hz rate
 * 
 * Safely reads the current cycle_rate_hz under spinlock protection and converts
 * to milliseconds. Ensures minimum 1ms period to prevent divide-by-zero.
 * Used by traffic thread for timing LED state changes.
 * 
 * Return: Period in milliseconds (minimum 1ms)
 */
static inline int period_ms(void)
{
    /* return the cycle period in milliseconds; minimum 1 ms */
    int hz;
    unsigned long flags;

    spin_lock_irqsave(&state_lock, flags);
    hz = cycle_rate_hz;
    spin_unlock_irqrestore(&state_lock, flags);

    if (hz < 1) hz = 1;
    /* integer division, ensure at least 1 */
    if ((1000 / hz) < 1) return 1;
    return 1000 / hz;
}

static inline void led_set(int gpio, int on)
{
    /* set LED value if the GPIO is valid */
    if (gpio_is_valid(gpio))
        gpio_set_value(gpio, (on ? 1 : 0));
}


static inline void all_off(void)
{
    /* turn all LEDs off */
    led_set(red_gpio, 0);
    led_set(yellow_gpio, 0);
    led_set(green_gpio, 0);
}

// static functions to control all three LED Colors as a function of states (show_green(), show_yellow(), show_red(), show_red_yelllow())
//      all of one function sets all three LEDs to a specified colors. 
static inline void show_green(void)  { led_set(green_gpio, 1); led_set(yellow_gpio, 0); led_set(red_gpio, 0); }
static inline void show_yellow(void) { led_set(green_gpio, 0); led_set(yellow_gpio, 1); led_set(red_gpio, 0); }
static inline void show_red(void)    { led_set(green_gpio, 0); led_set(yellow_gpio, 0); led_set(red_gpio, 1); }
static inline void show_red_yellow(void) { led_set(green_gpio, 0); led_set(yellow_gpio, 1); led_set(red_gpio, 1); }


/* ------------------ Revised ISRs (spinlock, debounce, minimal work) ------------------ */

/**
 * btn_mode_isr() - Interrupt handler for BTN0 (mode switching button)
 * @irq: IRQ number
 * @dev_id: Device ID (unused)
 * 
 * Cycles through traffic light modes: NORMAL -> FLASH_RED -> FLASH_YELLOW -> NORMAL.
 * Implements debouncing to prevent spurious mode changes from button bouncing.
 * Uses spinlock to safely update shared mode variable from interrupt context.
 * Only processes button release (high level) to avoid repeated triggering.
 * 
 * Return: IRQ_HANDLED
 */
static irqreturn_t btn_mode_isr(int irq, void *dev_id)
{
    unsigned long now = jiffies;
    /* debounce */
    if (time_before(now, last_mode_jiffies + msecs_to_jiffies(debounce_ms)))
        return IRQ_HANDLED;
    last_mode_jiffies = now;

    /* read pin quickly; assume active-low (0 == pressed) */
    if (gpio_get_value(btn_mode_gpio) == 1) {
        unsigned long flags;
        spin_lock_irqsave(&state_lock, flags);
        /* cycle modes (safe single-word op) */
        mode = (mode == MODE_FLASH_YELLOW) ? MODE_NORMAL : (mode + 1);
        spin_unlock_irqrestore(&state_lock, flags);
        pr_info("mytraffic: ISR mode -> %d\n", (int)mode);
    }
    return IRQ_HANDLED;
}

/**
 * btn_ped_isr() - Interrupt handler for BTN1 (pedestrian call button)
 * @irq: IRQ number  
 * @dev_id: Device ID (unused)
 * 
 * Sets pedestrian waiting flag when button is pressed during normal mode.
 * This causes the next red phase to show red+yellow for 5 cycles instead of
 * red-only for 2 cycles, following Massachusetts traffic light convention.
 * Includes debouncing and only activates in normal mode.
 * 
 * Return: IRQ_HANDLED
 */
static irqreturn_t btn_ped_isr(int irq, void *dev_id)
{
    unsigned long now = jiffies;
    if (time_before(now, last_ped_jiffies + msecs_to_jiffies(debounce_ms)))
        return IRQ_HANDLED;
    last_ped_jiffies = now;

    if (gpio_get_value(btn_ped_gpio) == 1) { /* pressed (active-low) */
        unsigned long flags;
        spin_lock_irqsave(&state_lock, flags);
        if (mode == MODE_NORMAL)
            ped_waiting = true;  /* set flag, will be consumed by thread */
        spin_unlock_irqrestore(&state_lock, flags);
        pr_info("mytraffic: ISR ped_waiting set\n");
    }
    return IRQ_HANDLED;
}

/* ------------------ Revised traffic thread (no sleeping while locked) ------------------ */

/**
 * traffic_thread() - Main traffic light control thread
 * @arg: Thread argument (unused)
 * 
 * Implements the core traffic light state machine:
 * - Normal mode: Green(3) -> Yellow(1) -> Red(2) or Red+Yellow(5 if pedestrian)
 * - Flash modes: Alternate on/off at cycle rate
 * - Extra credit: Lightbulb check mode when both buttons held
 * 
 * Samples shared state under spinlock protection to avoid races, then operates
 * on local copies. Handles mode changes and pedestrian requests asynchronously.
 * Continues until module unload or kthread_should_stop().
 * 
 * Return: 0 on successful completion
 */
static int traffic_thread(void *arg)
{
    unsigned long loop_count = 0;

    while (!kthread_should_stop() && running) {
        int pm = period_ms();
        enum mode_t cur_mode;
        bool cur_ped_waiting;

        /* sample the shared state quickly under spinlock */
        {
            unsigned long flags;
            spin_lock_irqsave(&state_lock, flags);
            cur_mode = mode;
            cur_ped_waiting = ped_waiting;
            spin_unlock_irqrestore(&state_lock, flags);
        }

        pr_info("mytraffic: loop %lu mode=%d ped=%d pm=%d\n",
                ++loop_count, (int)cur_mode, (int)cur_ped_waiting, pm);

        /* read raw button snapshots locally when needed (avoid global stale vars) */
        {
            int mode_btn = gpio_get_value(btn_mode_gpio);
            int ped_btn  = gpio_get_value(btn_ped_gpio);

            /* Extra credit "lightbulb check" mode: both held */
            if (mode_btn == 1 && ped_btn == 1) {
                show_red();
                led_set(yellow_gpio, 1);
                led_set(green_gpio, 1);

                /* wait while both held */
                while (!kthread_should_stop() && running) {
                    if (gpio_get_value(btn_mode_gpio) != 1 || gpio_get_value(btn_ped_gpio) != 1)
                        break;
                    msleep(30);
                }

                /* reset state safely */
                {
                    unsigned long flags;
                    spin_lock_irqsave(&state_lock, flags);
                    mode = MODE_NORMAL;
                    cycle_rate_hz = 1;
                    ped_waiting = false;
                    spin_unlock_irqrestore(&state_lock, flags);
                }
                all_off();
                msleep(150);
                continue;
            }
        }

        /* state machine: operate based on sampled cur_mode and cur_ped_waiting.
           Any changes to mode/ped_waiting may happen via ISRs and will be re-sampled
           at the top of the next loop iteration. */

        switch (cur_mode) {
        case MODE_NORMAL:
            /* green 3 cycles */
            show_green();
            msleep(pm * 3);
            pr_info("mytraffic: after green\n");

            /* yellow 1 cycle */
            show_yellow();
            msleep(pm * 1);
            pr_info("mytraffic: after yellow\n");

            /* decide stop-phase using the flag sampled earlier.
               If the flag was set by ISR, consume it here (use lock just for assignment). */
            if (cur_ped_waiting) {
                /* consume request */
                {
                    unsigned long flags;
                    spin_lock_irqsave(&state_lock, flags);
                    /* only clear if still true to avoid races */
                    if (ped_waiting) ped_waiting = false;
                    spin_unlock_irqrestore(&state_lock, flags);
                }
                show_red_yellow();
                msleep(pm * 5);
                pr_info("mytraffic: after red+yellow (ped)\n");
            } else {
                show_red();
                msleep(pm * 2);
                pr_info("mytraffic: after red\n");
            }
            break;

        case MODE_FLASH_RED:
            show_red();
            msleep(pm);
            led_set(red_gpio, 0);
            msleep(pm);
            break;

        case MODE_FLASH_YELLOW:
            show_yellow();
            msleep(pm);
            led_set(yellow_gpio, 0);
            msleep(pm);
            break;

        default:
            /* unexpected mode — reset to normal safely */
            {
                unsigned long flags;
                spin_lock_irqsave(&state_lock, flags);
                mode = MODE_NORMAL;
                spin_unlock_irqrestore(&state_lock, flags);
            }
            break;
        }
        /* loop back and re-sample current state (mode/ped_waiting) */
    }

    all_off();
    pr_info("mytraffic: traffic_thread exiting\n");
    return 0;
}

// =======================================================================================================================
// Basic kernal module read/write/init/exit functions that implement overall functions for copy from usr and copy to user
// =======================================================================================================================

/* ------------------ Revised char device read/write using spinlock ------------------ */

/**
 * my_read() - Character device read operation for /dev/mytraffic
 * @f: File pointer
 * @ubuf: User buffer to write data to
 * @len: Length of user buffer
 * @off: File offset pointer
 * 
 * Returns current traffic light status including:
 * - Operating mode (normal/flashing-red/flashing-yellow)  
 * - Cycle rate in Hz
 * - Current LED states (on/off for red, yellow, green)
 * - Pedestrian status (present/absent)
 * 
 * Implements one-shot read behavior (returns 0 on subsequent reads).
 * Thread-safe using spinlock to sample shared state.
 * 
 * Return: Number of bytes read, or negative error code
 */
static ssize_t my_read(struct file *f, char __user *ubuf, size_t len, loff_t *off)
{
    char kbuf[256];
    int n, r, y, g, rate;
    enum mode_t m;
    bool ped;

    if (*off) return 0; /* one-shot read behavior preserved */

    /* sample shared state under spinlock */
    {
        unsigned long flags;
        spin_lock_irqsave(&state_lock, flags);
        m = mode;
        rate = cycle_rate_hz;
        ped = ped_waiting;
        spin_unlock_irqrestore(&state_lock, flags);
    }

    r = gpio_get_value(red_gpio);
    y = gpio_get_value(yellow_gpio);
    g = gpio_get_value(green_gpio);

    n = scnprintf(kbuf, sizeof(kbuf),
        "mode=%s\nrate=%d Hz\nstatus=red %s, yellow %s, green %s\npedestrian=%s\n",
        (m==MODE_NORMAL) ? "normal" :
            (m==MODE_FLASH_RED) ? "flashing-red" : "flashing-yellow",
        rate,
        r ? "on" : "off", y ? "on" : "off", g ? "on" : "off",
        ped ? "present" : "absent");

    if (n > (int)len) n = len; /* be safe */
    if (copy_to_user(ubuf, kbuf, n)) return -EFAULT;
    *off = n;
    return n;
}

/**
 * my_write() - Character device write operation for /dev/mytraffic
 * @f: File pointer
 * @ubuf: User buffer containing data to write
 * @len: Length of data to write
 * @off: File offset pointer
 * 
 * Accepts integer values 1-9 to set traffic light cycle rate in Hz.
 * For example, writing "3" sets 3Hz rate (333ms cycles).
 * Invalid input is silently ignored per assignment requirements.
 * Thread-safe using spinlock protection.
 * 
 * Return: Number of bytes written (always returns len)
 */
static ssize_t my_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
    char kbuf[16];
    int n = (len < (sizeof(kbuf)-1)) ? len : (int)sizeof(kbuf)-1;
    int v;

    if (copy_from_user(kbuf, ubuf, n)) return -EFAULT;
    kbuf[n] = '\0';

    if (kstrtoint(kbuf, 10, &v) == 0 && v >= 1 && v <= 9) {
        unsigned long flags;
        spin_lock_irqsave(&state_lock, flags);
        cycle_rate_hz = v;
        spin_unlock_irqrestore(&state_lock, flags);
        pr_info("mytraffic: cycle_rate_hz set to %d\n", v);
    }
    return len;
}


static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .read  = my_read,
    .write = my_write,
};

// gpio helpers 
/**
 * setup_gpio() - Configure a GPIO pin for input or output
 * @gpio: GPIO number to configure
 * @name: Descriptive name for the GPIO (for debugging)
 * @output: true for output, false for input
 * @initval: Initial value if configuring as output
 * 
 * Requests the GPIO from the kernel, sets direction, and initial value.
 * Handles validation and cleanup on error.
 * 
 * Return: 0 on success, negative error code on failure
 */
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


/* ------------------ Revised init/exit (spinlock init, proper class/device handling, safer IRQ setup) ------------------ */

/**
 * my_init() - Module initialization function
 * 
 * Sets up the complete traffic light system:
 * - Initializes spinlock for thread safety
 * - Configures GPIO pins for LEDs (outputs) and buttons (inputs)  
 * - Registers interrupt handlers for both buttons with debouncing
 * - Creates character device /dev/mytraffic (major 61, minor 0)
 * - Sets up device class and creates device node
 * - Starts the traffic control kernel thread
 * 
 * Implements proper error handling with cleanup on failure.
 * 
 * Return: 0 on success, negative error code on failure
 */
static int __init my_init(void)
{
    int ret;

    /* initialize spinlock */
    spin_lock_init(&state_lock);

    /* LEDs start OFF buttons are inputs */
    if ((ret = setup_gpio(red_gpio,      "led_red",    true,  0))) goto err_gpio;
    if ((ret = setup_gpio(yellow_gpio,   "led_yellow", true,  0))) goto err_gpio;
    if ((ret = setup_gpio(green_gpio,    "led_green",  true,  0))) goto err_gpio;
    if ((ret = setup_gpio(btn_mode_gpio, "btn_mode",   false, 0))) goto err_gpio;
    if ((ret = setup_gpio(btn_ped_gpio,  "btn_ped",    false, 0))) goto err_gpio;

    /* request IRQs: obtain irq numbers separately for clearer error handling */
    irq_mode = gpio_to_irq(btn_mode_gpio);
    if (irq_mode < 0) { pr_err("mytraffic: gpio_to_irq(mode) failed: %d\n", irq_mode); ret = irq_mode; goto err_gpio; }
    ret = request_irq(irq_mode, btn_mode_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mytraffic_btn_mode", NULL);
    if (ret) { pr_err("mytraffic: request_irq(mode) failed: %d\n", ret); goto err_gpio; }

    irq_ped = gpio_to_irq(btn_ped_gpio);
    if (irq_ped < 0) { pr_err("mytraffic: gpio_to_irq(ped) failed: %d\n", irq_ped); ret = irq_ped; goto err_irq_mode; }
    ret = request_irq(irq_ped, btn_ped_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mytraffic_btn_ped", NULL);
    if (ret) { pr_err("mytraffic: request_irq(ped) failed: %d\n", ret); goto err_irq_mode; }

    /* char device */
    ret = register_chrdev(MAJOR_NUM, DEVICE_NAME, &fops);
    if (ret < 0) { pr_err("mytraffic: register_chrdev failed: %d\n", ret); goto err_irq_both; }

    /* create class/device and keep pointers in module-scope vars */
    my_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(my_class)) { ret = PTR_ERR(my_class); pr_err("mytraffic: class_create failed\n"); goto err_chrdev; }

    my_dev = device_create(my_class, NULL, MKDEV(MAJOR_NUM, 0), NULL, DEVICE_NAME);
    if (IS_ERR(my_dev)) { ret = PTR_ERR(my_dev); pr_err("mytraffic: device_create failed\n"); class_destroy(my_class); my_class = NULL; goto err_chrdev; }

    /* start the timing thread */
    running = true;
    traffic_task = kthread_run(traffic_thread, NULL, "mytraffic_thread");
    if (IS_ERR(traffic_task)) {
        ret = PTR_ERR(traffic_task);
        pr_err("mytraffic: kthread_run failed: %d\n", ret);
        goto err_device;
    }

    pr_info("mytraffic: loaded (R=%d Y=%d G=%d | BTN0=%d BTN1=%d) — /dev/%s (major=%d)\n",
            red_gpio, yellow_gpio, green_gpio, btn_mode_gpio, btn_ped_gpio,
            DEVICE_NAME, MAJOR_NUM);
    return 0;

/* error cleanup paths (reverse order of allocation) */
err_device:
    if (my_dev && !IS_ERR(my_dev)) device_destroy(my_class, MKDEV(MAJOR_NUM, 0));
    if (my_class && !IS_ERR(my_class)) class_destroy(my_class);
err_chrdev:
    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
err_irq_both:
    if (irq_ped >= 0) { free_irq(irq_ped, NULL); irq_ped = -1; }
err_irq_mode:
    if (irq_mode >= 0) { free_irq(irq_mode, NULL); irq_mode = -1; }
err_gpio:
    all_off();
    free_gpios();
    return ret;
}

/**
 * my_exit() - Module cleanup function
 * 
 * Safely shuts down the traffic light system:
 * - Stops the traffic control thread
 * - Frees interrupt handlers  
 * - Unregisters character device
 * - Destroys device class and device node
 * - Turns off all LEDs and releases GPIO pins
 * 
 * Ensures clean shutdown without resource leaks.
 */
static void __exit my_exit(void)
{
    running = false;
    if (traffic_task && !IS_ERR(traffic_task))
        kthread_stop(traffic_task);

    if (irq_mode >= 0) { free_irq(irq_mode, NULL); irq_mode = -1; }
    if (irq_ped  >= 0) { free_irq(irq_ped,  NULL); irq_ped = -1; }

    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);

    if (my_dev && !IS_ERR(my_dev)) {
        device_destroy(my_class, MKDEV(MAJOR_NUM, 0));
        my_dev = NULL;
    }
    if (my_class && !IS_ERR(my_class)) {
        class_destroy(my_class);
        my_class = NULL;
    }

    /* turn all LEDs off and free gpios */
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
