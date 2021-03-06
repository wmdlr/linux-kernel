/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio_dma"

void spi_dma_init (struct dw_spi *dws);

struct dw_spi_mmio_dma {
    struct dw_spi  dws;
    struct clk    *clk;
};

static int dw_spi_mmio_dma_probe(struct platform_device *pdev)
{
    struct dw_spi_mmio_dma *dwsmmio;
    struct dw_spi *dws;
    struct resource *mem;
    int ret;
    int num_cs;

    dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio_dma), GFP_KERNEL);
    if (!dwsmmio) {
        return -ENOMEM;
    }

    dws = &dwsmmio->dws;

    /* Get basic io resource and map it */
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!mem) {
        dev_err(&pdev->dev, "no mem resource?\n");
        return -EINVAL;
    }

    dws->regs = devm_ioremap_resource(&pdev->dev, mem);
    if (IS_ERR(dws->regs)) {
        dev_err(&pdev->dev, "SPI region map failed\n");
        return PTR_ERR(dws->regs);
    }
    dws->paddr = dws->regs;

    dws->irq = platform_get_irq(pdev, 0);
    if (dws->irq < 0) {
        dev_err(&pdev->dev, "no irq resource?\n");
        return dws->irq; /* -ENXIO */
    }

    dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(dwsmmio->clk)) {
        return PTR_ERR(dwsmmio->clk);
    }
    ret = clk_prepare_enable(dwsmmio->clk);
    if (ret) {
        return ret;
    }

    dws->bus_num = of_alias_get_id(pdev->dev.of_node, "ssi");
    dws->max_freq = clk_get_rate(dwsmmio->clk);
    device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);
    device_property_read_u32(&pdev->dev, "num-cs", &dws->num_cs);

    if (pdev->dev.of_node) {
        int i;
        for (i = 0; i < dws->num_cs; i++) {
            int cs_gpio = of_get_named_gpio(pdev->dev.of_node, "cs-gpios", i);

            if (cs_gpio == -EPROBE_DEFER) {
                ret = cs_gpio;
                goto out;
            }

            if (gpio_is_valid(cs_gpio)) {
                ret = devm_gpio_request(&pdev->dev, cs_gpio, dev_name(&pdev->dev));
                if (ret)
                    goto out;
            }
        }
    }

    spi_dma_init(dws);

    ret = dw_spi_add_host(&pdev->dev, dws);
    if (ret)
        goto out;

    platform_set_drvdata(pdev, dwsmmio);
    return 0;

out:
    clk_disable_unprepare(dwsmmio->clk);
    return ret;
}

static int dw_spi_mmio_dma_remove(struct platform_device *pdev)
{
    struct dw_spi_mmio_dma *dwsmmio = platform_get_drvdata(pdev);

    dw_spi_remove_host(&dwsmmio->dws);
    clk_disable_unprepare(dwsmmio->clk);

    return 0;
}

static const struct of_device_id dw_spi_mmio_dma_of_match[] = {
    { .compatible = "snps,dw-apb-ssi-dma", },
    { /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_dma_of_match);

static struct platform_driver dw_spi_mmio_dma_driver = {
    .probe      = dw_spi_mmio_dma_probe,
    .remove     = dw_spi_mmio_dma_remove,
    .driver     = {
        .name   = DRIVER_NAME,
        .of_match_table = dw_spi_mmio_dma_of_match,
    },
};
module_platform_driver(dw_spi_mmio_dma_driver);

MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
