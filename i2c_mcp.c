/* An i2c  driver for MCP794xx RTC */
#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc/ds1307.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>


#define DS1307_REG_WDAY 0x03
#define MCP794xx_BIT_VBATEN 0x08
#define MCP794XX_REG_CONTROL 0x07
#define MCP794XX_BIT_ALM0_EN 0x10
#define MCP794XX_BIT_ALM1_EN 0x20
#define MCP794XX_REG_ALARM0_BASE 0x0a
#define MCP794XX_REG_ALARM0_CTRL 0x0d
#define MCP794XX_REG_ALARM1_BASE 0x11
#define MCP794XX_REG_ALARM1_CTRL 0x14
#define MCP794XX_BIT_ALMX_IF BIT(3)
#define MCP794XX_BIT_ALMX_C0 BIT(4)
#define MCP794XX_BIT_ALMX_C1 BIT(5)
#define MCP794XX_BIT_ALMX_C2 BIT(6)
#define MCP794XX_BIT_ALMX_POL BIT(7)
#define MCP794XX_MSK_ALMX_MATCH	(MCP794XX_BIT_ALMX_C0 | \
					 MCP794XX_BIT_ALMX_C1 | \
					 MCP794XX_BIT_ALMX_C2)
/* Choose the interrupt pin as GPIO 19 i.e pin 35 on rpi 3 */
#define GPIO_ANY_GPIO 19
#define GPIO_ANY_GPIO_DESC "Some gpio pin description"
#define GPIO_ANY_GPIO_DEVICE_DESC "some_device"

/* Function prototyping section */
static int mcp794xx_read_alarm(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t mcp794xx_set_alarm(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
int mcp794xx_open(struct inode *inode, struct file *filp);
void i2c_do_tasklet(struct work_struct *work);


/* Global Variables section */
/* class variable */
static struct class *c1;
static unsigned int mcp_major = 1;
static unsigned int minor = 0;
/* Interrupt Variable */
short int irq_any_gpio;
static struct work_struct i2c_wq;

struct mcp794xx {
	//enum ds_type	type;
	unsigned long	flags;
#define HAS_NVRAM	0		/* bit 0 == sysfs file active */
#define HAS_ALARM	1		/* bit 1 == irq claimed */
	struct device	*dev;
	struct regmap	*regmap;
	const char	*name;
	struct rtc_device *rtc;
	struct cdev cdev;
	int current_pointer;
#ifdef CONFIG_COMMON_CLK
	struct clk_hw	clks[2];
#endif
};


struct chip_desc {
	unsigned		alarm:1;
	u16			nvram_offset;
	u16			nvram_size;
	u8			offset; /* register's offset */
	u8			century_reg;
	u8			century_enable_bit;
	u8			century_bit;
	u8			bbsqi_bit;
	irq_handler_t		irq_handler;
	const struct rtc_class_ops *rtc_ops;
	u16			trickle_charger_reg;
	u8			(*do_trickle_setup)(struct mcp794xx *, u32,
						    bool);
};

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
struct file_operations mcp794xx_fops = {
	
	.owner = THIS_MODULE,
	.read = mcp794xx_read_alarm,
	.write = mcp794xx_set_alarm,
	.open = mcp794xx_open,
	
		
};
static int mcp794xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mcp794xx	*mcp794xx;
	struct device *device = NULL;
	dev_t devno = 0;
	int err = -ENODEV;
	
	//bool want_irq;
	//bool mcp794xx_can_wakeup_device = false;
	//unsigned char regs[8];
	

	mcp794xx = devm_kzalloc(&client->dev, sizeof(struct mcp794xx), GFP_KERNEL);
	if (mcp794xx == NULL)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, mcp794xx);
	mcp794xx->dev = &client->dev;
	mcp794xx->name = client->name;

	mcp794xx->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(mcp794xx->regmap)) {
		dev_err(mcp794xx->dev, "regmap allocation failed\n");
		return PTR_ERR(mcp794xx->regmap);
	}
	mcp_major = MAJOR(devno);
	/* Create device class */
	c1 = class_create(THIS_MODULE, "mcp794xx");
	if(IS_ERR(c1))
	{
		err = PTR_ERR(c1);		
		goto fail;
	}
	
	cdev_init(&mcp794xx->cdev, &mcp794xx_fops);
	err = cdev_add(&mcp794xx->cdev, devno, 1);
	if (err) {
		pr_err("Error while trying to add %s\n", "MCP794xx");
		goto fail;
	}
	/* Create device with NULL parent device and NULL child device */
	device = device_create(c1, NULL, devno, NULL, "MCP794xx");
	if(IS_ERR(device)) {
		err = PTR_ERR(device);
		pr_err("error while creating device %s", "MCP794xx");
		cdev_del(&mcp794xx->cdev);
		goto fail;
	}
	INIT_WORK(&i2c_wq, i2c_do_tasklet);
	
	i2c_set_clientdata(client, mcp794xx);
	
	return 0;
fail:
	if (c1 != NULL) {
		device_destroy(c1, MKDEV(mcp_major, minor));
		class_destroy(c1);				
	}
	return err;	
}


static int mcp794xx_remove(struct i2c_client *client)
{
	/* free created device */
	device_destroy(c1, MKDEV(mcp_major, 1));
	/* free created class */
	class_destroy(c1);
	return 0;
}


/* Called when device is opened for the first time */
int mcp794xx_open(struct inode *inode, struct file *filp)
{
	struct mcp794xx *dev = NULL;
	dev = container_of(inode->i_cdev, struct mcp794xx, cdev);
	if (dev == NULL) {
		pr_err("Container of didn't find any valid data \n");
		return -ENODEV;
	}

	dev->current_pointer = 0;

	filp->private_data = dev;
	
	if(inode->i_cdev != &dev->cdev) {

		pr_err("Device open: internal error\n");
		return -ENODEV;
	}
	dev = (struct mcp794xx *) devm_kzalloc(dev->dev, sizeof(struct mcp794xx *), GFP_KERNEL);
	if(dev == NULL) {
		pr_err("Error allocating memory \n");
		return -ENOMEM;
	}
	return 0;
}
ssize_t mcp794xx_set_alarm(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct device *dev = NULL;
	struct rtc_wkalrm *t = NULL;
	struct mcp794xx *mcp794xx = dev_get_drvdata(dev);
	u8 regs[10];
	int wday = 0, ret;

	//buffer = devm_kzalloc(mcp794xx->dev, 10*sizeof(char *), GFP_KERNEL);

	/* Use sysfs here */
	dev_dbg(dev, "%s, sec=%d min=%d hour=%d wday=%d mday=%d mon=%d "
		"enabled=%d pending=%d\n", __func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
		t->time.tm_wday, t->time.tm_mday, t->time.tm_mon,
		t->enabled, t->pending);
	
	/* Read control registers */
	ret = regmap_bulk_read(mcp794xx->regmap, MCP794XX_REG_CONTROL, regs, sizeof(regs));

	if (ret)
		return ret;

	/* Set alarm 0, using 24-hour and day-of-month modes. */
	regs[3] = bin2bcd(t->time.tm_sec);
	regs[4] = bin2bcd(t->time.tm_min);
	regs[5] = bin2bcd(t->time.tm_hour);
	regs[6] = wday;
	regs[7] = bin2bcd(t->time.tm_mday);
	regs[8] = bin2bcd(t->time.tm_mon + 1);

	/* Clear the alarm 0 interrupt flag. */
	regs[6] &= ~MCP794XX_BIT_ALMX_IF;
	/* Set alarm match: second, minute, hour, day, date, month. */
	regs[6] |= MCP794XX_MSK_ALMX_MATCH;
	/* Disable interrupt. We will not enable until completely programmed */
	regs[0] &= ~MCP794XX_BIT_ALM0_EN;

	/* write the values to set to the control register */
	ret = regmap_bulk_write(mcp794xx->regmap, MCP794XX_REG_CONTROL, regs,
				sizeof(regs));
	if (ret < 0)
		return -EIO;
	
	return ret;
}
static int mcp794xx_read_alarm(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct device *dev = NULL;
	//struct rtc_wkalrm *t;
	u8 regs[10];
	int ret;

	struct mcp794xx *mcp794xx = dev_get_drvdata(dev);
	/* used to Store read values from registers */
	buf = devm_kzalloc(mcp794xx->dev, 10*sizeof(char *), GFP_KERNEL);
	

	ret = regmap_bulk_read(mcp794xx->regmap, MCP794XX_REG_CONTROL, regs, sizeof(regs));
	
	if(ret)
		return ret;
	
	if (copy_to_user(buf, regs, 10) != 0) {

		return -EIO;
	}
	
	return 0;
}

void i2c_do_tasklet(struct work_struct *work)
{
	struct device *dev = NULL;
	//struct rtc_wkalrm *t;
	u8 regs[10];
	int ret;

	struct mcp794xx *mcp794xx = dev_get_drvdata(dev);
	/* read control register of rtc device */
	ret = regmap_bulk_read(mcp794xx->regmap, MCP794XX_REG_CONTROL, regs, sizeof(regs));


}


/* Interrupt Handling section */
/* IRQ handler - fired on GPIO 17 interrupt- Falling Edge */
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{

	/* Schedule the Bottom Half to be executed later */
	schedule_work(&i2c_wq);
	
	/* Respond to the interrupt */
	return IRQ_HANDLED;	
}
/* Module to configure interrupt */
void r_int_config(void)
{
	if (gpio_request(GPIO_ANY_GPIO, GPIO_ANY_GPIO_DESC)) {
		printk("GPIO request failure %s\n", GPIO_ANY_GPIO_DESC);
		return;	
	}
	/* Obtain the irq number for our desired interrupt gpio pin */
	if ( (irq_any_gpio = gpio_to_irq(GPIO_ANY_GPIO)) < 0) {
		printk("GPIO to IRQ mapping failure %s\n", GPIO_ANY_GPIO_DESC);
		return;
	}	
	
	printk(KERN_NOTICE "Mapped int %d\n", irq_any_gpio);
	/* Configure irq handler to Trigger on Falling Edge */	
	if (request_irq(irq_any_gpio, (irq_handler_t) r_irq_handler, IRQF_TRIGGER_FALLING, GPIO_ANY_GPIO_DESC, GPIO_ANY_GPIO_DEVICE_DESC )) {
		printk("Irq Request failure \n");
		return;
	}
	return;	
}
/* Module to release interrupt resources configured */
void r_int_release(void) 
{
	free_irq(irq_any_gpio, GPIO_ANY_GPIO_DEVICE_DESC);
	gpio_free(GPIO_ANY_GPIO);
	
}

static const struct i2c_device_id mcp794xx_id[] = {

	{ "mcp7940x", 12 },
	{},
};

MODULE_DEVICE_TABLE(i2c, mcp794xx_id);


#ifdef CONFIG_OF
static const struct of_device_id mcp794xx_of_match[] = {
	
	{
		.compatible = "microchip,mcp7940x",
		//.data = (void *)mcp794xx
	},
	{}	


};
MODULE_DEVICE_TABLE(of, mcp794xx_of_match);
#endif

static struct i2c_driver mcp794xx_driver = {
	.driver = {
		.name	= "rtc-mcp794xx",
		.of_match_table = of_match_ptr(mcp794xx_of_match),
		//.acpi_match_table = ACPI_PTR(mcp794xx_acpi_ids),
	},
	.probe = mcp794xx_probe,
	.remove = mcp794xx_remove,
	.id_table = mcp794xx_id,
	
};

module_i2c_driver(mcp794xx_driver);
MODULE_DESCRIPTION("Device driver for rtc chip MCP794xx ");
MODULE_LICENSE("GPL");
