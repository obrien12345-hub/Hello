#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/semaphore.h>
#include <linux/timekeeping.h>
#include <linux/ioctl.h>

#define DEVICE_NAME "iot_sensor"
#define LOG_DEVICE_NAME "iot_log"

#define MAX_SIZE 64
#define LOG_SIZE 1024
#define MAX_LOG_ENTRIES 50

#define IOC_MAGIC 'k'
#define IOC_SET_SENSOR _IOW(IOC_MAGIC, 1, int)
#define IOC_SET_UNITS  _IOW(IOC_MAGIC, 2, int)
#define IOC_SET_RANGE  _IOW(IOC_MAGIC, 3, struct sensor_range)

enum sensor_type { TEMP = 0, HUMIDITY = 1, PRESSURE = 2 };
enum temp_unit { CELSIUS = 0, FAHRENHEIT = 1 };

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

    struct log_entry logs[MAX_LOG_ENTRIES];
    int log_index;

    struct cdev sensor_cdev;
    struct cdev log_cdev;
};

static struct iot_device dev;
static dev_t sensor_dev;
static dev_t log_dev;
static struct class *iot_class;

/* ================= FILE OPS ================= */

static int device_open(struct inode *inode, struct file *file) {
    if (down_interruptible(&dev.sem))
        return -ERESTARTSYS;
    return 0;
}

static int device_release(struct inode *inode, struct file *file) {
    up(&dev.sem);
    return 0;
}

static ssize_t device_read(struct file *file, char __user *buffer,
                           size_t len, loff_t *offset) {
    char data[MAX_SIZE];
    int value;

    if (*offset > 0)
        return 0;

    if ((get_random_u32() % 100) < 5)
        return -EIO;

    get_random_bytes(&value, sizeof(value));
    value = dev.range.min +
            (value % (dev.range.max - dev.range.min + 1));

    switch (dev.type) {
    case TEMP:
        snprintf(data, MAX_SIZE, "Temperature: %d %s\n",
                 value, dev.unit == CELSIUS ? "C" : "F");
        break;
    case HUMIDITY:
        snprintf(data, MAX_SIZE, "Humidity: %d %%\n", value);
        break;
    case PRESSURE:
        snprintf(data, MAX_SIZE, "Pressure: %d hPa\n", value);
        break;
    }

    snprintf(dev.logs[dev.log_index].data, MAX_SIZE, "%s", data);
    dev.logs[dev.log_index].timestamp = ktime_get();
    dev.log_index = (dev.log_index + 1) % MAX_LOG_ENTRIES;

    if (copy_to_user(buffer, data, strlen(data)))
        return -EFAULT;

    *offset = strlen(data);
    return strlen(data);
}

static long device_ioctl(struct file *file,
                         unsigned int cmd, unsigned long arg) {
    int val;
    struct sensor_range r;

    switch (cmd) {
    case IOC_SET_SENSOR:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
            return -EFAULT;
        if (val >= 0 && val <= 2)
            dev.type = val;
        else
            return -EINVAL;
        break;

    case IOC_SET_UNITS:
        if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
            return -EFAULT;
        if (val == 0 || val == 1)
            dev.unit = val;
        else
            return -EINVAL;
        break;

    case IOC_SET_RANGE:
        if (copy_from_user(&r, (struct sensor_range __user *)arg, sizeof(r)))
            return -EFAULT;
        if (r.min < r.max)
            dev.range = r;
        else
            return -EINVAL;
        break;

    default:
        return -ENOTTY;
    }

    return 0;
}

static ssize_t log_read(struct file *file, char __user *buffer,
                        size_t len, loff_t *offset) {
    char logbuf[LOG_SIZE];
    int i, pos = 0;

    if (*offset > 0)
        return 0;

    for (i = 0; i < MAX_LOG_ENTRIES && pos < LOG_SIZE - MAX_SIZE; i++) {
        pos += snprintf(logbuf + pos, LOG_SIZE - pos,
                        "[%lld] %s",
                        ktime_to_ms(dev.logs[i].timestamp),
                        dev.logs[i].data);
    }

    if (copy_to_user(buffer, logbuf, pos))
        return -EFAULT;

    *offset = pos;
    return pos;
}

/* ================= FILE OPS TABLES ================= */

static struct file_operations sensor_fops = {
    .owner = THIS_MODULE,
    .open = device_open,
    .release = device_release,
    .read = device_read,
    .unlocked_ioctl = device_ioctl,
};

static struct file_operations log_fops = {
    .owner = THIS_MODULE,
    .read = log_read,
};

/* ================= INIT / EXIT ================= */

static int __init iot_init(void) {
    sema_init(&dev.sem, 1);

    dev.type = TEMP;
    dev.unit = CELSIUS;
    dev.range.min = 0;
    dev.range.max = 50;
    dev.log_index = 0;

    alloc_chrdev_region(&sensor_dev, 0, 1, DEVICE_NAME);
    alloc_chrdev_region(&log_dev, 0, 1, LOG_DEVICE_NAME);

    cdev_init(&dev.sensor_cdev, &sensor_fops);
    cdev_init(&dev.log_cdev, &log_fops);

    cdev_add(&dev.sensor_cdev, sensor_dev, 1);
    cdev_add(&dev.log_cdev, log_dev, 1);

    iot_class = class_create(THIS_MODULE, "iot");

    device_create(iot_class, NULL, sensor_dev, NULL, DEVICE_NAME);
    device_create(iot_class, NULL, log_dev, NULL, LOG_DEVICE_NAME);

    printk(KERN_INFO "iot_sensor loaded\n");
    return 0;
}

static void __exit iot_exit(void) {
    device_destroy(iot_class, sensor_dev);
    device_destroy(iot_class, log_dev);

    class_destroy(iot_class);

    cdev_del(&dev.sensor_cdev);
    cdev_del(&dev.log_cdev);

    unregister_chrdev_region(sensor_dev, 1);
    unregister_chrdev_region(log_dev, 1);

    printk(KERN_INFO "iot_sensor unloaded\n");
}

module_init(iot_init);
module_exit(iot_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OS Lab");
MODULE_DESCRIPTION("Virtual IoT Sensor Driver");
