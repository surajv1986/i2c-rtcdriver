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
/* Function prototyping section */
static int mcp794xx_read_alarm(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t mcp794xx_set_alarm(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

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
	//.open = mcp794xx_open,
	//.close = mcp794xx_close,
		
};
static int mcp794xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mcp794xx	*mcp794xx;
	int err = -ENODEV;
	int tmp;
	bool want_irq;
	bool mcp794xx_can_wakeup_device = false;
	unsigned char	regs[8];
	struct mcp794xx_platform_data *pdata = dev_get_platdata(&client->dev);
	u8 trickle_charger_setup = 0;

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

	cdev_init(&mcp794xx->cdev, &mcp794xx_fops);
	i2c_set_clientdata(client, mcp794xx);

	return 0;	
}

ssize_t mcp794xx_set_alarm(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct device *dev;
	struct rtc_wkalrm *t;
	char *buff;
	struct mcp794xx *mcp794xx = dev_get_drvdata(dev);
	u8 ald[3], ctl[3];
	u8 regs[10];
	int wday, ret;

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
	struct mcp794xx *mcp794xx = dev_get_drvdata(dev);
	/* used to Store read values from registers */
	buf = devm_kzalloc(mcp794xx->dev, 10*sizeof(char *), GFP_KERNEL);
	u8 regs[10];
	int ret;

	ret = regmap_bulk_read(mcp794xx->regmap, MCP794XX_REG_CONTROL, regs, sizeof(regs));
	
	if(ret)
		return ret;
	
	if (copy_to_user(buf, regs, 10) != 0) {

		return -EIO;
	}
	
	return 0;
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
	.id_table = mcp794xx_id,
	//.remove = mcp794xx_remove,
};

module_i2c_driver(mcp794xx_driver);
MODULE_DESCRIPTION("Device driver for rtc chip MCP794xx ");
MODULE_LICENSE("GPL");
