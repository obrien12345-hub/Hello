// iot_sensor.c  (Linux 6.x compatible)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/random.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/string.h>

#define DEV_BASENAME     "iot_sensor"
#define SENSOR_DEVNAME   "iot_sensor"
#define LOG_DEVNAME      "iot_log"

#define MAX_SIZE         64
#define LOG_SIZE         2048
#define MAX_LOG_ENTRIES  64

// IOCTL
#define IOC_MAGIC        'k'
#define IOC_SET_SENSOR   _IOW(IOC_MAGIC, 1, int)                  // 0 temp, 1 humidity, 2 pressure
#define IOC_SET_UNITS    _IOW(IOC_MAGIC, 2, int)                  // 0 C, 1 F (temp only)
#define IOC_SET_RANGE    _IOW(IOC_MAGIC, 3, struct sensor_range)  // min/max
#define IOC_RESET_LOG    _IO(IOC_MAGIC, 4)                        // reset log ring + counters

// Sensor types
enum sensor_type { TEMP = 0, HUMIDITY = 1, PRESSURE = 2 };
enum temp_unit   { CELSIUS = 0, FAHRENHEIT = 1 };

// IMPORTANT: rename to avoid kernel's struct range
struct sensor_range {
    int min;
    int max;
};

struct log_entry {
    char data[MAX_SIZE];
    ktime_t timestamp;
};

struct iot_device {
    enum sensor_type type;
    enum temp_unit unit;
    struct sensor_range range;

    struct semaphore sem;

    bool override_active;
    int  override_value;

    struct log_entry logs[MAX_LOG_ENTRIES];
    int log_head;
    int log_count;

    u64 total_read_ns;
    u64 read_count;

    dev_t dev_base;               // base dev number for 2 minors
    struct cdev sensor_cdev;
    struct cdev log_cdev;
    struct class *cls;
};

static struct iot_device gdev;

static void add_logf(const char *fmt, ...)
{
    va_list args;
    char tmp[MAX_SIZE];
    int idx;

    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    idx = gdev.log_head;

    // Use snprintf (portable) instead of strlcpy to avoid implicit decl issues
    snprintf(gdev.logs[idx].data, MAX_SIZE, "%s", tmp);
    gdev.logs[idx].timestamp = ktime_get();

    gdev.log_head = (gdev.log_head + 1) % MAX_LOG_ENTRIES;
    if (gdev.log_count < MAX_LOG_ENTRIES)
        gdev.log_count++;
}

static int random_in_range(int min, int max)
{
    u32 r;
    int span;

    if (min > max) {
        int t = min; min = max; max = t;
    }
    span = (max - min) + 1;
    get_random_bytes(&r, sizeof(r));
    if (span <= 0) return min;
    return min + (r % span);
}

// ---------- Sensor device ops ----------
static int sensor_open(struct inode *inode, struct file *file)
{
    if (down_interruptible(&gdev.sem))
        return -ERESTARTSYS;
    return 0;
}

static int sensor_release(struct inode *inode, struct file *file)
{
    up(&gdev.sem);
    return 0;
}

static ssize_t sensor_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
    char data[MAX_SIZE];
    int value;
    size_t bytes_to_copy;
    ktime_t t0, t1;
    u64 dt;

    if (*offset >= MAX_SIZE)
        return 0;

    // 5% chance of sensor failure
    if (random_in_range(0, 99) < 5) {
        add_logf("ERROR: Simulated sensor failure (EIO)\n");
        return -EIO;
    }

    t0 = ktime_get();

    if (gdev.override_active) {
        value = gdev.override_value;
    } else {
        value = random_in_range(gdev.range.min, gdev.range.max);
    }

    switch (gdev.type) {
        case TEMP:
            snprintf(data, MAX_SIZE, "Temperature: %d %s\n", value,
                     (gdev.unit == CELSIUS) ? "C" : "F");
            break;
        case HUMIDITY:
            snprintf(data, MAX_SIZE, "Humidity: %d%%\n", value);
            break;
        case PRESSURE:
        default:
            snprintf(data, MAX_SIZE, "Pressure: %d hPa\n", value);
            break;
    }

    add_logf("%s", data);

    t1 = ktime_get();
    dt = (u64)ktime_to_ns(ktime_sub(t1, t0));
    gdev.total_read_ns += dt;
    gdev.read_count++;

    bytes_to_copy = min(length, (size_t)(MAX_SIZE - *offset));
    if (copy_to_user(buffer, data + *offset, bytes_to_copy))
        return -EFAULT;

    *offset += bytes_to_copy;
    return bytes_to_copy;
}

static ssize_t sensor_write(struct file *filp, const char __user *buffer, size_t length, loff_t *offset)
{
    char kbuf[MAX_SIZE];
    long v;
    size_t n = min(length, (size_t)(MAX_SIZE - 1));

    if (copy_from_user(kbuf, buffer, n))
        return -EFAULT;
    kbuf[n] = '\0';

    if (kstrtol(kbuf, 10, &v) != 0)
        return -EINVAL;

    if (v < gdev.range.min || v > gdev.range.max)
        return -EINVAL;

    gdev.override_active = true;
    gdev.override_value = (int)v;
    add_logf("Manual set: %d\n", (int)v);

    return (ssize_t)n;
}

static long sensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int v;
    struct sensor_range r;

    switch (cmd) {
        case IOC_SET_SENSOR:
            if (copy_from_user(&v, (int __user *)arg, sizeof(int)))
                return -EFAULT;
            if (v < 0 || v > 2)
                return -EINVAL;
            gdev.type = (enum sensor_type)v;
            add_logf("Config: sensor_type=%d\n", v);
            return 0;

        case IOC_SET_UNITS:
            if (copy_from_user(&v, (int __user *)arg, sizeof(int)))
                return -EFAULT;
            if (v < 0 || v > 1)
                return -EINVAL;
            if (gdev.type != TEMP)
                return -EINVAL;
            gdev.unit = (enum temp_unit)v;
            add_logf("Config: units=%d\n", v);
            return 0;

        case IOC_SET_RANGE:
            if (copy_from_user(&r, (struct sensor_range __user *)arg, sizeof(struct sensor_range)))
                return -EFAULT;
            if (r.min >= r.max)
                return -EINVAL;
            gdev.range = r;
            gdev.override_active = false;
            add_logf("Config: range=[%d,%d]\n", r.min, r.max);
            return 0;

        case IOC_RESET_LOG:
            gdev.log_head = 0;
            gdev.log_count = 0;
            gdev.total_read_ns = 0;
            gdev.read_count = 0;
            gdev.override_active = false;
            add_logf("Logs reset\n");
            return 0;

        default:
            return -ENOTTY;
    }
}

// ---------- Log device ops ----------
static ssize_t log_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
    char *out;
    int i, start, idx;
    int written = 0;

    out = kzalloc(LOG_SIZE, GFP_KERNEL);
    if (!out)
        return -ENOMEM;

    // Performance metric line
    if (gdev.read_count > 0) {
        u64 avg_ns = gdev.total_read_ns / gdev.read_count;
        written += scnprintf(out + written, LOG_SIZE - written,
                             "AVG_READ_MS: %llu.%03llu\n",
                             avg_ns / 1000000ULL,
                             (avg_ns % 1000000ULL) / 1000ULL);
    } else {
        written += scnprintf(out + written, LOG_SIZE - written, "AVG_READ_MS: N/A\n");
    }

    // Logs oldest -> newest
    start = (gdev.log_head - gdev.log_count + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
    for (i = 0; i < gdev.log_count && written < (LOG_SIZE - MAX_SIZE); i++) {
        idx = (start + i) % MAX_LOG_ENTRIES;
        written += scnprintf(out + written, LOG_SIZE - written,
                             "[%lld] %s",
                             (long long)ktime_to_ms(gdev.logs[idx].timestamp),
                             gdev.logs[idx].data);
    }

    if (*offset >= written) {
        kfree(out);
        return 0;
    }

    {
        size_t to_copy = min(length, (size_t)(written - *offset));
        if (copy_to_user(buffer, out + *offset, to_copy)) {
            kfree(out);
            return -EFAULT;
        }
        *offset += to_copy;
        kfree(out);
        return (ssize_t)to_copy;
    }
}

static const struct file_operations sensor_fops = {
    .owner          = THIS_MODULE,
    .open           = sensor_open,
    .release        = sensor_release,
    .read           = sensor_read,
    .write          = sensor_write,
    .unlocked_ioctl = sensor_ioctl,
};

static const struct file_operations log_fops = {
    .owner = THIS_MODULE,
    .read  = log_read,
};

static int __init iot_init(void)
{
    int ret;

    sema_init(&gdev.sem, 1);
    gdev.type = TEMP;
    gdev.unit = CELSIUS;
    gdev.range.min = 0;
    gdev.range.max = 50;
    gdev.override_active = false;
    gdev.log_head = 0;
    gdev.log_count = 0;
    gdev.total_read_ns = 0;
    gdev.read_count = 0;

    // allocate 2 minors: 0 sensor, 1 log
    ret = alloc_chrdev_region(&gdev.dev_base, 0, 2, DEV_BASENAME);
    if (ret < 0)
        return ret;

    // Linux 6.x: class_create takes only (name)
    gdev.cls = class_create("iot");
    if (IS_ERR(gdev.cls)) {
        unregister_chrdev_region(gdev.dev_base, 2);
        return PTR_ERR(gdev.cls);
    }

    // sensor minor 0
    cdev_init(&gdev.sensor_cdev, &sensor_fops);
    ret = cdev_add(&gdev.sensor_cdev, gdev.dev_base + 0, 1);
    if (ret < 0)
        goto fail1;

    device_create(gdev.cls, NULL, gdev.dev_base + 0, NULL, SENSOR_DEVNAME);

    // log minor 1
    cdev_init(&gdev.log_cdev, &log_fops);
    ret = cdev_add(&gdev.log_cdev, gdev.dev_base + 1, 1);
    if (ret < 0)
        goto fail2;

    device_create(gdev.cls, NULL, gdev.dev_base + 1, NULL, LOG_DEVNAME);

    add_logf("Driver initialized (major=%d)\n", MAJOR(gdev.dev_base));
    printk(KERN_INFO "iot_sensor: loaded major=%d\n", MAJOR(gdev.dev_base));
    return 0;

fail2:
    device_destroy(gdev.cls, gdev.dev_base + 0);
    cdev_del(&gdev.sensor_cdev);
fail1:
    class_destroy(gdev.cls);
    unregister_chrdev_region(gdev.dev_base, 2);
    return ret;
}

static void __exit iot_exit(void)
{
    device_destroy(gdev.cls, gdev.dev_base + 1);
    cdev_del(&gdev.log_cdev);

    device_destroy(gdev.cls, gdev.dev_base + 0);
    cdev_del(&gdev.sensor_cdev);

    class_destroy(gdev.cls);
    unregister_chrdev_region(gdev.dev_base, 2);

    printk(KERN_INFO "iot_sensor: unloaded\n");
}

module_init(iot_init);
module_exit(iot_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OS Lab");
MODULE_DESCRIPTION("Advanced Virtual IoT Multi-Sensor Driver");
