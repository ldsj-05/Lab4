/* mytraffic.c — EC535 Lab 4 “Traffic Light” kernel module
 *
 * Adds brief explanatory comments and TODO markers + stub functions
 * for the remaining lab requirements: GPIO control, IRQs, timing/state,
 * readable/writable char device, and extra credit “bulb check”.
 */

/* --- Includes: core driver headers --- */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>      /* printk() */
#include <linux/slab.h>        /* kmalloc()/kfree() */
#include <linux/fs.h>          /* register_chrdev(), file_operations */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/uaccess.h>     /* copy_to_user()/copy_from_user() */

/* --- Includes: timing, IRQ, GPIO, kthread (some may be unused yet) --- */
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>        /* legacy integer GPIO API (allowed) */
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>

/* NOTE: Older/duplicate headers removed (e.g., <asm/uaccess.h>), keep code modern. */

MODULE_LICENSE("Dual BSD/GPL");

/* --- GPIO assignments (BBB header numbers -> kernel GPIO numbers)
 * These are placeholders; confirm pin mux + numeric IDs using the BBB docs.
 * TODO: verify these numeric GPIO IDs against your wiring.
 */
#define RED     67
#define YELLOW  68
#define GREEN   44
#define BTN0    26
#define BTN1    46   /* FIX: BTN146 looked like a typo; set to real BTN1 line */

/* --- Buffer sizing for char device I/O --- */
enum { capacity = 4096, bite = 128 };

/* --- Forward declarations: file ops --- */
static int     mytraffic_open(struct inode *inode, struct file *filp);
static int     mytraffic_release(struct inode *inode, struct file *filp);
static ssize_t mytraffic_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t mytraffic_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

/* --- Forward declarations: module init/exit --- */
static int  mytraffic_init(void);
static void mytraffic_exit(void);

/* --- TODO Forward declarations: lab features --- */
/* GPIO and IRQ lifecycle */
static int  tl_gpio_request_config(void);     /* TODO: request and set directions for RED/YEL/GRN + BTN0/BTN1 */
static void tl_gpio_free(void);               /* TODO: free requested GPIOs */
static int  tl_irq_request(void);             /* TODO: request IRQs for BTN0/BTN1 (falling/rising as needed) */
static void tl_irq_free(void);                /* TODO: free IRQs */

/* Light control + state machine + timing */
static void tl_all_off(void);                 /* TODO: helper to switch all LEDs off */
static void tl_set_red(int on);               /* TODO: drive RED LED */
static void tl_set_yellow(int on);            /* TODO: drive YELLOW LED */
static void tl_set_green(int on);             /* TODO: drive GREEN LED */

static void tl_tick(struct timer_list *t);    /* TODO: periodic “cycle” timer callback */
static void tl_schedule_next_tick(unsigned hz);

/* Mode and pedestrian logic */
typedef enum { MODE_NORMAL = 0, MODE_FLASH_RED, MODE_FLASH_YELLOW, MODE_COUNT } tl_mode_t;
static void tl_mode_next(void);               /* TODO: BTN0 handler: cycle modes */
static void tl_ped_request(void);             /* TODO: BTN1 handler: pedestrian call */
static void tl_apply_state_step(void);        /* TODO: advance normal sequence (G3, Y1, R2 or R+Y 5 if ped) */

/* Readout builder for /dev/mytraffic */
static int  tl_build_status(char *dst, size_t maxlen);  /* TODO: format mode, rate, light states, ped-present */

/* Write handler helper */
static void tl_set_rate_from_digit(unsigned d);         /* TODO: set cycle rate 1..9 Hz */

/* Extra credit */
static int  tl_check_bulb_mode_active(void);            /* TODO: both buttons held? */

/* --- File operations table (legacy style OK for this course) --- */
static struct file_operations mytraffic_fops = {
  .read    = mytraffic_read,
  .write   = mytraffic_write,
  .open    = mytraffic_open,
  .release = mytraffic_release,
};

/* --- Module entry/exit wiring --- */
module_init(mytraffic_init);
module_exit(mytraffic_exit);

/* --- Globals: char device bookkeeping --- */
static int   mytraffic_major = 61;    /* Per lab spec: use major 61, minor 0 */
static char *mytraffic_buffer;        /* backing store for /dev reads/writes (status, input) */
static int   mytraffic_len;

/* --- Globals: IRQ bookkeeping & runtime state --- */
static int irq0 = -1, irq1 = -1;      /* assigned by request_irq */
static atomic_t ped_waiting = ATOMIC_INIT(0);  /* pedestrian requested flag */

/* Mode and rate (Hz) — defaults per spec */
static tl_mode_t mode = MODE_NORMAL;
static unsigned  cycle_hz = 1;        /* writable via /dev/mytraffic write(“1..9”) */

/* --- Timer for “cycles” (1 Hz default) --- */
static DEFINE_TIMER(tl_timer, tl_tick);

/* ========== Module init/exit ========== */

static int mytraffic_init(void)
{
  int result;

  /* Register character device: creates major number for /dev/mytraffic */
  result = register_chrdev(mytraffic_major, "mytraffic", &mytraffic_fops);
  if (result < 0) {
    printk(KERN_ALERT "mytraffic: cannot obtain major %d\n", mytraffic_major);
    return result;
  }

  /* Allocate a simple kernel buffer for read/write demo + status text */
  mytraffic_buffer = kmalloc(capacity, GFP_KERNEL);
  if (!mytraffic_buffer) {
    printk(KERN_ALERT "mytraffic: insufficient kernel memory\n");
    result = -ENOMEM;
    goto fail_chrdev;
  }
  memset(mytraffic_buffer, 0, capacity);
  mytraffic_len = 0;

  /* TODO: request + configure GPIOs (LEDs out, buttons in w/ pull-ups if needed) */
  result = tl_gpio_request_config();
  if (result) {
    printk(KERN_ALERT "mytraffic: gpio request/config failed (%d)\n", result);
    goto fail_buf;
  }

  /* TODO: request IRQs for BTN0/BTN1 to handle mode switch and ped call */
  result = tl_irq_request();
  if (result) {
    printk(KERN_ALERT "mytraffic: irq request failed (%d)\n", result);
    goto fail_gpio;
  }

  /* Initialize lights to a known state (all off) */
  tl_all_off();

  /* Start periodic timer ticking at cycle_hz (1 Hz default) */
  tl_schedule_next_tick(cycle_hz);

  printk(KERN_INFO "mytraffic: module inserted (mode=normal, rate=%u Hz)\n", cycle_hz);
  return 0;

fail_gpio:
  tl_gpio_free();
fail_buf:
  kfree(mytraffic_buffer);
fail_chrdev:
  unregister_chrdev(mytraffic_major, "mytraffic");
  return result;
}

static void mytraffic_exit(void)
{
  /* Stop timer before tearing down GPIO/IRQs */
  del_timer_sync(&tl_timer);

  /* Free IRQs and GPIOs */
  tl_irq_free();
  tl_gpio_free();

  /* Unregister char device major */
  unregister_chrdev(mytraffic_major, "mytraffic");

  /* Free buffer */
  kfree(mytraffic_buffer);

  printk(KERN_INFO "mytraffic: module removed\n");
}

/* ========== File operations ========== */

static int mytraffic_open(struct inode *inode, struct file *filp)
{
  /* Simple open: no per-file state yet.
   * TODO: if you add private state, set filp->private_data here.
   */
  return 0;
}

static int mytraffic_release(struct inode *inode, struct file *filp)
{
  /* Simple release: nothing to tear down. */
  return 0;
}

static ssize_t mytraffic_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  /* When read, return a human-readable snapshot of status:
   * - current mode
   * - current cycle rate (Hz)
   * - current LED states
   * - pedestrian present (if supported)
   * We’ll build it into mytraffic_buffer, then copy_to_user().
   */

  char tbuf[256];
  int n;

  /* Build fresh status string each read start */
  if (*f_pos == 0) {
    n = tl_build_status(tbuf, sizeof(tbuf));
    n = min(n, (int)capacity);
    memcpy(mytraffic_buffer, tbuf, n);
    mytraffic_len = n;
  }

  /* EOF if f_pos at end */
  if (*f_pos >= mytraffic_len)
    return 0;

  /* Do not exceed tail or user’s count */
  if (count > mytraffic_len - *f_pos)
    count = mytraffic_len - *f_pos;
  if (count > bite)
    count = bite;

  if (copy_to_user(buf, mytraffic_buffer + *f_pos, count))
    return -EFAULT;

  /* Debug trace (short) */
  printk(KERN_INFO "mytraffic: read %zu bytes @pos=%lld\n", count, *f_pos);

  *f_pos += count;
  return count;
}

static ssize_t mytraffic_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
  /* Write path: accept a single digit '1'..'9' to set cycle_hz.
   * Anything else is silently ignored (but won’t crash).
   */

  char kbuf[8];
  size_t c = min(count, sizeof(kbuf) - 1);

  if (c == 0)
    return 0;

  if (copy_from_user(kbuf, buf, c))
    return -EFAULT;

  kbuf[c] = '\0';

  /* Parse first digit if within 1..9 */
  if (kbuf[0] >= '1' && kbuf[0] <= '9') {
    unsigned d = kbuf[0] - '0';
    tl_set_rate_from_digit(d);
    printk(KERN_INFO "mytraffic: set rate to %u Hz via write\n", cycle_hz);
  } else {
    /* Silently ignore non-digit or out-of-range */
  }

  /* For completeness, store last write bytes (optional) */
  if (*f_pos >= capacity)
    return -ENOSPC;
  if (c > capacity - *f_pos)
    c = capacity - *f_pos;
  memcpy(mytraffic_buffer + *f_pos, kbuf, c);
  *f_pos += c;
  if (*f_pos > mytraffic_len) mytraffic_len = *f_pos;

  return count;
}

/* ========== TODO: GPIO/IRQ implementation stubs ========== */

/* NOTE: For this lab you may use legacy integer GPIO API (gpio_request, gpio_direction_*,
 * gpio_set_value, gpio_get_value) or the newer descriptor-based gpiod API.
 * Stubs below assume legacy integer API for brevity.
 */

static int tl_gpio_request_config(void)
{
  int ret = 0;

  /* TODO: request LEDs as outputs, default LOW */
  ret |= gpio_request(RED,    "tl_red");
  ret |= gpio_request(YELLOW, "tl_yellow");
  ret |= gpio_request(GREEN,  "tl_green");
  if (ret) goto err;

  ret |= gpio_direction_output(RED,    0);
  ret |= gpio_direction_output(YELLOW, 0);
  ret |= gpio_direction_output(GREEN,  0);
  if (ret) goto err;

  /* TODO: request BTN0/BTN1 as inputs (consider pull-ups via device tree or default state) */
  ret |= gpio_request(BTN0, "tl_btn0");
  ret |= gpio_request(BTN1, "tl_btn1");
  if (ret) goto err;

  ret |= gpio_direction_input(BTN0);
  ret |= gpio_direction_input(BTN1);
  if (ret) goto err;

  return 0;

err:
  printk(KERN_ERR "mytraffic: gpio request/config error\n");
  return ret ?: -EINVAL;
}

static void tl_gpio_free(void)
{
  /* Free in reverse order; ensure LEDs off */
  tl_all_off();
  gpio_free(BTN1);
  gpio_free(BTN0);
  gpio_free(GREEN);
  gpio_free(YELLOW);
  gpio_free(RED);
}

/* IRQ handlers — quick, defer work to timer/state where possible */
static irqreturn_t btn0_isr(int irq, void *dev_id)
{
  /* TODO: debounce if necessary (or rely on hardware) */
  tl_mode_next();                  /* cycle modes: normal -> flash-red -> flash-yellow -> normal */
  return IRQ_HANDLED;
}

static irqreturn_t btn1_isr(int irq, void *dev_id)
{
  /* TODO: debounce if necessary */
  tl_ped_request();                /* set ped_waiting flag; applied at next stop phase in normal mode */
  return IRQ_HANDLED;
}

static int tl_irq_request(void)
{
  int ret;

  /* Map GPIO to IRQ numbers */
  irq0 = gpio_to_irq(BTN0);
  irq1 = gpio_to_irq(BTN1);
  if (irq0 < 0 || irq1 < 0) return -EINVAL;

  /* TODO: choose trigger edges (falling/rising) matching wiring */
  ret  = request_irq(irq0, btn0_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mytraffic_btn0", NULL);
  ret |= request_irq(irq1, btn1_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mytraffic_btn1", NULL);
  return ret ? -EINVAL : 0;
}

static void tl_irq_free(void)
{
  if (irq0 >= 0) free_irq(irq0, NULL);
  if (irq1 >= 0) free_irq(irq1, NULL);
  irq0 = irq1 = -1;
}

/* ========== TODO: Light control helpers ========== */

static void tl_all_off(void)
{
  gpio_set_value(RED,    0);
  gpio_set_value(YELLOW, 0);
  gpio_set_value(GREEN,  0);
}

static void tl_set_red(int on)    { gpio_set_value(RED,    !!on); }
static void tl_set_yellow(int on) { gpio_set_value(YELLOW, !!on); }
static void tl_set_green(int on)  { gpio_set_value(GREEN,  !!on); }

/* ========== TODO: Timer + scheduling ========== */

static void tl_schedule_next_tick(unsigned hz)
{
  unsigned long interval = HZ / (hz ? hz : 1);
  mod_timer(&tl_timer, jiffies + interval);
}

/* Periodic callback: advances state machine each “cycle” and reschedules itself */
static void tl_tick(struct timer_list *t)
{
  /* Extra credit check: if both buttons held, light all and hold */
  if (tl_check_bulb_mode_active()) {
    tl_set_red(1); tl_set_yellow(1); tl_set_green(1);
    /* Do not advance state; reschedule */
    tl_schedule_next_tick(cycle_hz);
    return;
  }

  switch (mode) {
    case MODE_NORMAL:
      tl_apply_state_step();     /* implements Gx, Y1, R2 (or R+Y 5 if ped_waiting) */
      break;
    case MODE_FLASH_RED:
      /* toggle red each cycle */
      gpio_set_value(RED, !gpio_get_value(RED));
      gpio_set_value(YELLOW, 0);
      gpio_set_value(GREEN, 0);
      break;
    case MODE_FLASH_YELLOW:
      /* toggle yellow each cycle */
      gpio_set_value(YELLOW, !gpio_get_value(YELLOW));
      gpio_set_value(RED, 0);
      gpio_set_value(GREEN, 0);
      break;
    default:
      tl_all_off();
      break;
  }

  tl_schedule_next_tick(cycle_hz);
}

/* ========== TODO: Mode/pedestrian logic ========== */

static void tl_mode_next(void)
{
  mode = (mode + 1) % MODE_COUNT;
  /* Reset any per-mode counters if you keep them */
}

static void tl_ped_request(void)
{
  /* Only meaningful in normal mode; still set flag and consume at next stop */
  atomic_set(&ped_waiting, 1);
}

/* Implement a small internal step counter to realize:
 * normal: green 3 cycles -> yellow 1 -> red 2 (or red+yellow 5 if ped_waiting set), then repeat.
 * You can keep static counters inside this function or module-level.
 */
static void tl_apply_state_step(void)
{
  static int phase = 0;   /* 0..(variable) across the 3/1/2 (or 5) sequence */
  static int sub = 0;     /* ticks within current phase */

  /* Example sketch (fill in fully):
   * phase 0: green, for 3 ticks
   * phase 1: yellow, for 1 tick
   * phase 2: red (or red+yellow if ped_waiting), for 2 or 5 ticks
   */
  switch (phase) {
    case 0: /* green 3 */
      tl_set_green(1); tl_set_yellow(0); tl_set_red(0);
      if (++sub >= 3) { sub = 0; phase = 1; }
      break;
    case 1: /* yellow 1 */
      tl_set_green(0); tl_set_yellow(1); tl_set_red(0);
      if (++sub >= 1) { sub = 0; phase = 2; }
      break;
    case 2: { /* red 2 or red+yellow 5 if ped_waiting */
      int ped = atomic_xchg(&ped_waiting, 0); /* consume if set */
      if (ped) {
        tl_set_green(0); tl_set_yellow(1); tl_set_red(1);
        if (++sub >= 5) { sub = 0; phase = 0; }
      } else {
        tl_set_green(0); tl_set_yellow(0); tl_set_red(1);
        if (++sub >= 2) { sub = 0; phase = 0; }
      }
      break;
    }
    default:
      phase = 0; sub = 0;
      tl_all_off();
      break;
  }
}

/* ========== TODO: /dev readout + write helpers ========== */

static const char *mode_str(tl_mode_t m)
{
  switch (m) {
    case MODE_NORMAL:       return "normal";
    case MODE_FLASH_RED:    return "flashing-red";
    case MODE_FLASH_YELLOW: return "flashing-yellow";
    default:                return "unknown";
  }
}

static int tl_build_status(char *dst, size_t maxlen)
{
  int n = 0;
  int r = gpio_get_value(RED);
  int y = gpio_get_value(YELLOW);
  int g = gpio_get_value(GREEN);
  int ped = atomic_read(&ped_waiting);

  /* Format a single-line status string (OK for watch/cat) */
  n = scnprintf(dst, maxlen,
      "mode=%s rate=%uHz lights={red:%s,yellow:%s,green:%s} ped=%s\n",
      mode_str(mode), cycle_hz,
      r ? "on" : "off", y ? "on" : "off", g ? "on" : "off",
      ped ? "present" : "none");
  return n;
}

static void tl_set_rate_from_digit(unsigned d)
{
  /* Accept 1..9 Hz; clamp for safety */
  if (d < 1) d = 1;
  if (d > 9) d = 9;
  cycle_hz = d;
  /* Restart timer cadence immediately */
  tl_schedule_next_tick(cycle_hz);
}

/* ========== TODO: Extra credit bulb check ========== */

static int tl_check_bulb_mode_active(void)
{
  /* Return 1 while BOTH buttons are held; else 0.
   * Adjust for your wiring (active-low vs active-high).
   */
  int b0 = gpio_get_value(BTN0);
  int b1 = gpio_get_value(BTN1);
  /* Example assumes active-low buttons (pressed = 0) */
  return (b0 == 0 && b1 == 0);
}
