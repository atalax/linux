/*
 * Sunxi THS driver
 *
 * Copyright (C) 2015 Josef Gajdusek
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#define THS_H3_CTRL0			0x00
#define THS_H3_CTRL1			0x04
#define THS_H3_CDAT				0x14
#define THS_H3_CTRL2			0x40
#define THS_H3_INT_CTRL			0x44
#define THS_H3_STAT				0x48
#define THS_H3_ALARM_CTRL		0x50
#define THS_H3_SHUTDOWN_CTRL	0x60
#define THS_H3_FILTER			0x70
#define THS_H3_CDATA			0x74
#define THS_H3_DATA				0x80

#define THS_H3_CTRL0_SENSOR_ACQ0		0

#define THS_H3_CTRL1_ADC_CALI_EN		17
#define THS_H3_CTRL1_OP_BIAS			20

#define THS_H3_CTRL2_SENSE_EN			0
#define THS_H3_CTRL2_SENSOR_ACQ1		16

#define THS_H3_INT_CTRL_ALARM_INT_EN	0
#define THS_H3_INT_CTRL_SHUT_INT_EN		4
#define THS_H3_INT_CTRL_DATA_IRQ_EN		8
#define THS_H3_INT_CTRL_THERMAL_PER		12

#define THS_H3_STAT_ALARM_INT_STS		0
#define THS_H3_STAT_SHUT_INT_STS		4
#define THS_H3_STAT_DATA_IRQ_STS		8
#define THS_H3_STAT_ALARM_OFF_STS		12

#define THS_H3_ALARM_CTRL_ALARM0_T_HYST		0
#define THS_H3_ALARM_CTRL_ALARM0_T_HOT		16

#define THS_H3_SHUTDOWN_CTRL_SHUT0_T_HOT	16

#define THS_H3_FILTER_TYPE				0
#define THS_H3_FILTER_EN				2

struct sunxi_ths_data {
	struct sunxi_ths_type *type;
	struct reset_control *reset;
	struct clk *clk;
	struct clk *busclk;
	void __iomem *regs;
	void __iomem *calreg;
	struct platform_device *pdev;
	struct thermal_zone_device *tzd;
	int irq;
};

struct sunxi_ths_type {
	int (*init)(struct sunxi_ths_data *);
	int (*get_temp)(struct sunxi_ths_data *, int *out);
};

static int sunxi_ths_reg_to_temperature(int32_t reg, int divisor, int minus)
{
	return minus - (reg * 1000000) / divisor;
}

static int sunxi_ths_get_temp(void *_data, int *out)
{
	struct sunxi_ths_data *data = _data;

	return data->type->get_temp(data, out);
}

static int sunxi_ths_h3_init(struct sunxi_ths_data *data)
{
	if (data->calreg)
		writel(readl(data->calreg) & 0xfff, data->regs + THS_H3_CDATA);
	/* Magical constants mostly taken from Allwinner 3.4 kernel.
	 * Seem to work fine, though this could be configurable in DT/sysft
	 */
	writel(0xff << THS_H3_CTRL0_SENSOR_ACQ0,
			data->regs + THS_H3_CTRL0);
	writel((0x3f << THS_H3_CTRL2_SENSOR_ACQ1) | BIT(THS_H3_CTRL2_SENSE_EN),
			data->regs + THS_H3_CTRL2);
	writel((0x390 << THS_H3_INT_CTRL_THERMAL_PER) | BIT(THS_H3_INT_CTRL_DATA_IRQ_EN),
			data->regs + THS_H3_INT_CTRL);
	writel(BIT(THS_H3_FILTER_EN) | (0x2 << THS_H3_FILTER_TYPE),
			data->regs + THS_H3_FILTER);
	return 0;
}

static int sunxi_ths_h3_get_temp(struct sunxi_ths_data *data, int *out)
{
	*out = sunxi_ths_reg_to_temperature(readl(data->regs + THS_H3_DATA),
			8253, 217000);
	return 0;
}

static const struct thermal_zone_of_device_ops sunxi_ths_thermal_ops = {
	.get_temp = sunxi_ths_get_temp,
};

static const struct sunxi_ths_type sunxi_ths_device_h3 = {
	.init = sunxi_ths_h3_init,
	.get_temp = sunxi_ths_h3_get_temp,
};

static const struct of_device_id sunxi_ths_id_table[] = {
	{
		.compatible = "allwinner,sun8i-h3-ths",
		.data = &sunxi_ths_device_h3,
	},
	{
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, sunxi_ths_id_table);

static int sunxi_ths_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct sunxi_ths_data *data;
	struct resource *res;
	int ret;

	match = of_match_node(sunxi_ths_id_table, np);
	if (!match)
		return -ENXIO;

	data =
		devm_kzalloc(&pdev->dev, sizeof(struct sunxi_ths_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->type = (struct sunxi_ths_type *) match->data;
	data->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->regs)) {
		ret = PTR_ERR(data->regs);
		dev_err(&pdev->dev, "failed to ioremap THS registers: %d\n", ret);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	data->calreg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->calreg))
		data->calreg = NULL; /* No calibration register */

	data->busclk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(data->busclk)) {
		ret = PTR_ERR(data->busclk);
		dev_err(&pdev->dev, "failed to get ahb clk: %d\n", ret);
		return ret;
	}

	data->clk = devm_clk_get(&pdev->dev, "ths");
	if (IS_ERR(data->clk)) {
		ret = PTR_ERR(data->clk);
		dev_err(&pdev->dev, "failed to get ths clk: %d\n", ret);
		return ret;
	}

	data->reset = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(data->reset)) {
		ret = PTR_ERR(data->reset);
		dev_err(&pdev->dev, "failed to get reset: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(data->busclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable bus clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable ths clk: %d\n", ret);
		goto err_disable_bus;
	}

	ret = reset_control_deassert(data->reset);
	if (ret) {
		dev_err(&pdev->dev, "reset deassert failed: %d\n", ret);
		goto err_disable_ths;
	}

	ret = data->type->init(data);
	if (ret)
		goto err_reset_assert;

	data->tzd = thermal_zone_of_sensor_register(&pdev->dev, 0, data,
									&sunxi_ths_thermal_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		dev_err(&pdev->dev, "failed to register thermal zone: %d\n",
				ret);
		goto err_reset_assert;
	}

	platform_set_drvdata(pdev, data);
	return 0;

err_reset_assert:
	reset_control_assert(data->reset);
err_disable_ths:
	clk_disable_unprepare(data->clk);
err_disable_bus:
	clk_disable_unprepare(data->busclk);
	return ret;
}

static int sunxi_ths_remove(struct platform_device *pdev)
{
	struct sunxi_ths_data *data = platform_get_drvdata(pdev);

	thermal_zone_of_sensor_unregister(&pdev->dev, data->tzd);
	reset_control_assert(data->reset);
	clk_disable_unprepare(data->clk);
	clk_disable_unprepare(data->busclk);
	return 0;
}

static struct platform_driver sunxi_ths_driver = {
	.probe = sunxi_ths_probe,
	.remove = sunxi_ths_remove,
	.driver = {
		.name = "sunxi_ths",
		.of_match_table = sunxi_ths_id_table,
	},
};

module_platform_driver(sunxi_ths_driver);

MODULE_AUTHOR("Josef Gajdusek <atx@atx.name>");
MODULE_DESCRIPTION("Sunxi THS driver");
MODULE_LICENSE("GPL v2");
