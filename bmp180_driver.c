// Include library will be use
# include <linux/init.h>
# include <linux/module.h>
# include <linux/kernel.h>
# include <linux/init.h>
# include <linux/i2c.h>
# include <linux/delay.h>
# include <linux/types.h>

# define DRIVER_NAME "bmp180_driver"
# define CLASS_NAME "bmp180"
# define DEVICE_NAME "bmp180"

// Define the calibration register address, using for calculate pressure and temperature
# define BMP180_REG_AC1 0xAA
# define BMP180_REG_AC2 0xAC
# define BMP180_REG_AC3 0xAE
# define BMP180_REG_AC4 0xB0
# define BMP180_REG_AC5 0xB2
# define BMP180_REG_AC6 0xB4
# define BMP180_REG_B1 0xB6
# define BMP180_REG_B2 0xB8
# define BMP180_REG_MB 0xBA
# define BMP180_REG_MC 0xBC
# define BMP180_REG_MD 0xBE

// Define the control measurement register address
# define BMP180_MEASURE_CONTROL 0xF4

// Define the control register value, for measure temperature or measure pressure (pressure have 4 mode)
# define BMP180_TEMP 0x2E
# define BMP180_PRESS_ULTRA_LOW 0
# define BMP180_PRESS_STANDARD 1
# define BMP180_PRESS_HIGH_RESOLUTION 2
# define BMP180_PRESS_ULTRA_HIGH 3

// Define adc out register address
# define BMP180_ADC_MSB_ADDRESS 0xF6
# define BMP180_ADC_LSB_ADDRESS 0xF7
# define BMP180_ADC_XLSB_ADDRESS 0xF8

// Declaring variable to store calibration parameter, AC4 AC5 AC6 is unsigned short, the remain is short type
s16 AC1, AC2, AC3, B1, B2, MB, MC, MD ;
u16 AC4, AC5, AC6 ;

// Source code for BMP180 library
# define BMP180_IOCTL_MAGIC 'm'
# define BMP180_IOCTL_OSS _IOR(BMP180_IOCTL_MAGIC, 1, int)
# define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 2, int)
# define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 3, int)

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int major_number;

// Create a function to read a 16 bit register
static s16 bmp180_read_short(struct i2c_client *client, u8 register_address)
{
    s16 msb, lsb ;
    msb = i2c_smbus_read_byte_data(client, register_address);
    lsb = i2c_smbus_read_byte_data(client, register_address + 1);
    return (msb << 8) | lsb ;
}

static s16 bmp180_read_unsigned_short(struct i2c_client *client, u8 register_address)
{
    u16 msb, lsb ;
    msb = i2c_smbus_read_byte_data(client, register_address);
    lsb = i2c_smbus_read_byte_data(client, register_address + 1);
    return (msb << 8) | lsb ;
}

static void bmp180_read_calibration(struct i2c_client *client)
{
    // Read the calibration data using i2c library build in kernel
    AC1 = bmp180_read_short(client, BMP180_REG_AC1) ;
    AC2 = bmp180_read_short(client, BMP180_REG_AC2) ;
    AC3 = bmp180_read_short(client, BMP180_REG_AC3) ;
    AC4 = bmp180_read_unsigned_short(client, BMP180_REG_AC4) ;
    AC5 = bmp180_read_unsigned_short(client, BMP180_REG_AC5) ;
    AC6 = bmp180_read_unsigned_short(client, BMP180_REG_AC6) ;
    B1 = bmp180_read_short(client, BMP180_REG_B1) ;
    B2 = bmp180_read_short(client, BMP180_REG_B2) ;
    MB = bmp180_read_short(client, BMP180_REG_MB) ;    
    MC = bmp180_read_short(client, BMP180_REG_MC) ;
    MD = bmp180_read_short(client, BMP180_REG_MD) ;

}

static s32 bmp180_read_uncompensated_temp(struct i2c_client *client)
{
    s32 msb, lsb, UT ;
    i2c_smbus_write_byte_data(client, BMP180_MEASURE_CONTROL, BMP180_TEMP) ;
    msleep(5) ;
    msb = i2c_smbus_read_byte_data(client, BMP180_ADC_MSB_ADDRESS);
    lsb = i2c_smbus_read_byte_data(client, BMP180_ADC_LSB_ADDRESS);
    UT = (msb << 8) | lsb ;
    return UT ;
}

static s32 bmp180_read_uncompensated_press(struct i2c_client *client, int oss)
{
    s32 msb, lsb, xlsb, UP ;
    i2c_smbus_write_byte_data(client, BMP180_MEASURE_CONTROL, (0x34 + (oss << 6))) ;
    // Each mode (oss) have specific wait time to read register value
    switch (oss)
    {
    case 0: msleep(5) ; break ;
    case 1: msleep(8) ; break ;
    case 2: msleep(14) ; break ;
    case 3: msleep(26) ; break ;
    }
    msb = i2c_smbus_read_byte_data(client, BMP180_ADC_MSB_ADDRESS);
    lsb = i2c_smbus_read_byte_data(client, BMP180_ADC_LSB_ADDRESS);
    xlsb = i2c_smbus_read_byte_data(client, BMP180_ADC_XLSB_ADDRESS);

    UP = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - oss) ;

    return UP ;
}

static s32 bmp180_calculate_temp(s32 UT)
{
    s32 X1, X2, B5, t ;
    X1 = ((UT - AC6) * AC5) >> 15 ;
    X2 = (MC << 11) / (X1 + MD) ;
    B5 = X1 + X2 ;
    t = (B5 + 8) >> 4 ;
    return t ;
}

static s32 bmp180_calculate_press(s32 UT, s32 UP, int oss)
{
    s32 p, X1, X2, X3, B3, B4, B5, B6,  B7 ;
    X1 = ((UT - AC6) * AC5) >> 15 ;
    X2 = (MC << 11) / (X1 + MD) ;
    B5 = X1 + X2 ;
    B6 = B5 - 4000 ;
    X1 =  (B2 * ((B6 * B6) << 12)) >> 11 ;
    X2 = (AC2 * B6) >> 11 ;
    X3 = X1 + X2 ;
    B3 = (((AC1 * 4 + X3) << oss) + 2) >> 2 ;
    X1 = (AC3 * B6) >> 13 ;
    X2 = (B1 * ((B6 * B6) << 12)) >> 16 ;
    X3 = ((X1 + X2) + 2) >> 2 ;
    B4 = (AC4 * (s32) (X3 + 32768)) >> 15 ;
    B7 = ((u32) UP - B3) * (50000 >> oss) ;
    if (B7 < 0x80000000) {p = (B7 * 2) / B4 ;}
    else {p = (B7 / B4) * 2 ;}
    X1 = (p >> 8) * (p >> 8) ;
    X1 = (X1 * 3038) >> 16 ;
    X2 = (-7357 * p) >> 16 ;
    p = p + ((X1 + X2 + 3791) >> 2) ;
    return p  ;
}

static s32 bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    s32 data ;
    int oss ;
    oss = 0 ;

    switch (cmd) {
        case BMP180_IOCTL_OSS:
            if (copy_from_user(&oss, (int __user *)arg, sizeof(oss))) {
                return -EFAULT;
            }
            break;
        case BMP180_IOCTL_READ_TEMP:
            data = bmp180_calculate_temp(bmp180_read_uncompensated_temp(bmp180_client)) ;
            break;
        case BMP180_IOCTL_READ_PRESS:
            data = bmp180_calculate_press(bmp180_read_uncompensated_temp(bmp180_client), bmp180_read_uncompensated_press(bmp180_client, oss), oss) ;
            break;
        default:
            return -EINVAL;
    }

    if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
        return -EFAULT;
    }

    return 0;
}

static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device closed\n");
    return 0;
}

static struct file_operations fops = {
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
    .release = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    bmp180_client = client ;

    bmp180_read_calibration(client) ;

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    bmp180_class = class_create(CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(bmp180_device);
    }

    printk(KERN_INFO "BMP180 driver installed\n");
    
    return 0 ;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "BMP180 driver removed\n");
}

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { }
} ;

MODULE_DEVICE_TABLE(i2c, bmp180_id) ;

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = bmp180_probe,
    .remove     = bmp180_remove,
    .id_table   = bmp180_id,
};

static int __init bmp180_init(void)
{
    printk(KERN_INFO "Initializing BMP180 driver\n");
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
    printk(KERN_INFO "Exiting BMP180 driver\n");
    i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);
