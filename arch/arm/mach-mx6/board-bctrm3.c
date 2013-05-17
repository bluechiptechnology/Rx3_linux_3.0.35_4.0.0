/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/smsc911x.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/tlv320aic3x.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

//
// GPIO definitions
//
#define MX6Q_BCTRM3_SD1_CD          IMX_GPIO_NR(1, 1)
#define MX6Q_BCTRM3_SD1_WP          IMX_GPIO_NR(1, 9)
#define MX6Q_BCTRM3_ECSPI1_CS1      IMX_GPIO_NR(1, 13)

#define MX6Q_BCTRM3_CAN1_EN         IMX_GPIO_NR(2, 11)
#define MX6Q_BCTRM3_LED_CNTRL       IMX_GPIO_NR(7, 13)
#define MX6Q_BCTRM3_CSI0_RST        IMX_GPIO_NR(6, 15)

#define MX6Q_BCTRM3_AUDIO_SELECT    IMX_GPIO_NR(2, 14)
#define MX6Q_BCTRM3_EN_LITE         IMX_GPIO_NR(1, 3)
#define MX6Q_BCTRM3_EN_PANEL        IMX_GPIO_NR(4, 20)

#define MX6Q_BCTRM3_GPIO1           IMX_GPIO_NR(1, 4)
#define MX6Q_BCTRM3_GPIO2           IMX_GPIO_NR(4, 10)
#define MX6Q_BCTRM3_GPIO3           IMX_GPIO_NR(1, 25)
#define MX6Q_BCTRM3_GPIO4           IMX_GPIO_NR(1, 28)
#define MX6Q_BCTRM3_GPIO5           IMX_GPIO_NR(1, 27)
#define MX6Q_BCTRM3_GPIO6           IMX_GPIO_NR(1, 26)
#define MX6Q_BCTRM3_GPIO7           IMX_GPIO_NR(1, 30)
#define MX6Q_BCTRM3_GPIO8           IMX_GPIO_NR(1, 29)
#define MX6Q_BCTRM3_GPIO9           IMX_GPIO_NR(2, 12)
#define MX6Q_BCTRM3_GPIO10          IMX_GPIO_NR(1, 6)

#define MX6Q_BCTRM3_PER_RST         IMX_GPIO_NR(2, 8)
#define MX6Q_BCTRM3_WIRELESS_PWR_EN IMX_GPIO_NR(1, 2)

#define MX6Q_BCTRM3_SOM_IRQA_GPIO   IMX_GPIO_NR(6, 31)
#define MX6Q_BCTRM3_SOM_IRQB_GPIO   IMX_GPIO_NR(2, 28)
#define MX6Q_BCTRM3_SOM_IRQC_GPIO   IMX_GPIO_NR(2, 29)

//
// GPIO mappings
//
#define MX6Q_BCTRM3_TCH_INT1        MX6Q_BCTRM3_SOM_IRQB_GPIO
#define MX6Q_BCTRM3_USB_OTG_PWR     MX6Q_BCTRM3_GPIO10

//
// EIM Chip Selects
//
#define MX6Q_BCTRM3_EXPN_CE0        0   // EIM_CS0
#define MX6Q_BCTRM3_EXPN_CE         1   // EIM_CS1
#define MX6Q_BCTRM3_EXPN_CE4        2   // EIM_CS2


#define MX6Q_BCTRM3_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern char *gp_reg_id;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);

#define EIM_CSxGCR1 0x00	// Offset beyond EIM base register
#define EIM_CSxRCR1 0x08	// Offset beyond EIM base register
#define EIM_CSxWCR1 0x10	// Offset beyond EIM base register

#define CCM_CCGR6   0x80	// Offset beyond CCM base register
#define IOMC_GPR1   0x04	// Offset beyond IOMUXC base register

///
/// Sets up the EIM to have 4 chip selects, each with 32MB of address space
///
static void __init bctrm3_setup_eim(void)
{
	unsigned int value;
	void __iomem *ccm_base_reg = MX6_IO_ADDRESS(CCM_BASE_ADDR);
	void __iomem *iomux_base_reg = MX6_IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR);

	// Switch on EIM clocks
	value = readl(ccm_base_reg + CCM_CCGR6);
	value |= 	(3 << MXC_CCM_CCGRx_CG5_OFFSET);
	writel(value, ccm_base_reg + CCM_CCGR6);

	// Setup address spaces for CS0, CS1, CS2, CS3 (32MB each)
	value = readl(iomux_base_reg + IOMC_GPR1);
	value &= 0xfffff000;
	value |= 0x00000249;
	writel(value, iomux_base_reg + IOMC_GPR1);
}

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

#define SMSC911X_INTR_GPIO      MX6Q_BCTRM3_SOM_IRQC_GPIO
#define CS_SIZE                 0x2000000                 // 32MB

static struct resource smsc911x_resources[] = {
	[0] = {
		.name		= "smsc911x-memory",
		.start		= CS0_BASE_ADDR + (MX6Q_BCTRM3_EXPN_CE4 * CS_SIZE),
		.end		= CS0_BASE_ADDR + (MX6Q_BCTRM3_EXPN_CE4 * CS_SIZE) + 0xffff,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.name		= "smsc911x-irq",
		.start		= -1,				// Filled in by bctrm3_init_smsc911x()
		.end		= -1,				// unused
		.flags		= IORESOURCE_IRQ,
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type		= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags			= SMSC911X_USE_32BIT,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct platform_device smsc911x_device = {
	.name			= "smsc911x",
	.id				= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource		= smsc911x_resources,
	.dev = {
		.platform_data = &smsc911x_config,
	},
};

///
/// Sets up the specified CS with suitable timings for smsc911x chip
///
/// @param cs The chip select to be used (0-3)
///
static void __init bctrm3_setup_eimcs_for_smsc911x(int cs)
{
	unsigned int value;
	void __iomem *eim_base_reg = MX6_IO_ADDRESS(WEIM_BASE_ADDR);

	// Adjust address based on CS number (0-3)
	eim_base_reg += 0x18 * (cs);

	// Set up CS (16-bit async, non-multiplexed)
	value = (4 << 20) 	// CSREC
	      | (2 << 16)	// DSZ
	      | (1 << 7)	// CREP
	      | (1 << 0)	// CSEN
	      ;
	printk(KERN_DEBUG "CSxCGR1 = 0x%08x\n", value);
	__raw_writel(value, eim_base_reg + EIM_CSxGCR1);

	value = ( 7 << 24)	// RWSC
	      | ( 1 << 16)	// RADVN
	      | ( 1 << 12)	// OEA
	      | ( 0 <<  8)	// OEN
	      ;
	printk(KERN_DEBUG "CSxRCR1 = 0x%08x\n", value);
	__raw_writel(value, eim_base_reg + EIM_CSxRCR1);

	value = ( 7 << 24)	// WWSC
	      | ( 1 << 18)	// WADVN
	      | ( 1 <<  9)	// WEA
	      | ( 0 <<  6)	// WEN
	      ;
	printk(KERN_DEBUG "CSxWCR1 = 0x%08x\n", value);
	__raw_writel(value, eim_base_reg + EIM_CSxWCR1);
}

static inline void __init bctrm3_init_smsc911x(void)
{
	// Set up EIM chip select
	bctrm3_setup_eim();
	bctrm3_setup_eimcs_for_smsc911x(MX6Q_BCTRM3_EXPN_CE4);

	// Set up interrupt pin
	if (gpio_request(SMSC911X_INTR_GPIO, "smsc911x irq") < 0)
	{
		printk(KERN_ERR "Failed to request GPIO%d for smsc911x IRQ\n", SMSC911X_INTR_GPIO);
		return;
	}

	gpio_direction_input(SMSC911X_INTR_GPIO);
	smsc911x_resources[1].start = gpio_to_irq(SMSC911X_INTR_GPIO);
	smsc911x_resources[1].flags |=	(IORESOURCE_IRQ_LOWLEVEL & IRQF_TRIGGER_MASK);

	// Register the platform device
	platform_device_register(&smsc911x_device);
}
#else
static inline void __init bctrm3_init_smsc911x(void) { return; }
#endif

static iomux_v3_cfg_t mx6q_bctrm3_pads[] = {

	// AUDIO (IIS)
	MX6Q_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX6Q_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX6Q_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX6Q_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	MX6Q_PAD_SD4_DAT6__GPIO_2_14,                // AUDIO_SELECT

	// CAN
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,
	MX6Q_PAD_SD4_DAT3__GPIO_2_11,                // CAN_3

	// EIM
	MX6Q_PAD_EIM_OE__WEIM_WEIM_OE,
	MX6Q_PAD_EIM_RW__WEIM_WEIM_RW,
	MX6Q_PAD_EIM_CS0__WEIM_WEIM_CS_0,
	MX6Q_PAD_EIM_CS1__WEIM_WEIM_CS_1,
	MX6Q_PAD_SD2_DAT1__WEIM_WEIM_CS_2,
	MX6Q_PAD_EIM_D16__WEIM_WEIM_D_16,
	MX6Q_PAD_EIM_D17__WEIM_WEIM_D_17,
	MX6Q_PAD_EIM_D18__WEIM_WEIM_D_18,
	MX6Q_PAD_EIM_D19__WEIM_WEIM_D_19,
	MX6Q_PAD_EIM_D20__WEIM_WEIM_D_20,
	MX6Q_PAD_EIM_D21__WEIM_WEIM_D_21,
	MX6Q_PAD_EIM_D22__WEIM_WEIM_D_22,
	MX6Q_PAD_EIM_D23__WEIM_WEIM_D_23,
	MX6Q_PAD_EIM_D24__WEIM_WEIM_D_24,
	MX6Q_PAD_EIM_D25__WEIM_WEIM_D_25,
	MX6Q_PAD_EIM_D26__WEIM_WEIM_D_26,
	MX6Q_PAD_EIM_D27__WEIM_WEIM_D_27,
	MX6Q_PAD_EIM_D28__WEIM_WEIM_D_28,
	MX6Q_PAD_EIM_D29__WEIM_WEIM_D_29,
	MX6Q_PAD_EIM_D30__WEIM_WEIM_D_30,
	MX6Q_PAD_EIM_D31__WEIM_WEIM_D_31,
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,                 // EXPN_READY
	MX6Q_PAD_EIM_DA0__WEIM_WEIM_DA_A_0,
	MX6Q_PAD_EIM_DA1__WEIM_WEIM_DA_A_1,
	MX6Q_PAD_EIM_DA2__WEIM_WEIM_DA_A_2,
	MX6Q_PAD_EIM_DA3__WEIM_WEIM_DA_A_3,
	MX6Q_PAD_EIM_DA4__WEIM_WEIM_DA_A_4,
	MX6Q_PAD_EIM_DA5__WEIM_WEIM_DA_A_5,
	MX6Q_PAD_EIM_DA6__WEIM_WEIM_DA_A_6,
	MX6Q_PAD_EIM_DA10__WEIM_WEIM_DA_A_10,

	// ENET
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_GPIO_19__GPIO_4_5,                  // ETH_IRQ
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,

	// GPIO
	MX6Q_PAD_GPIO_4__GPIO_1_4,                   // CONN_GPIO1
	MX6Q_PAD_KEY_COL2__GPIO_4_10,                // CONN_GPIO2
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25,             // CONN_GPIO3
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,              // CONN_GPIO4
	MX6Q_PAD_ENET_RXD0__GPIO_1_27,               // CONN_GPIO5
	MX6Q_PAD_ENET_RXD1__GPIO_1_26,               // CONN_GPIO6
	MX6Q_PAD_ENET_TXD0__GPIO_1_30,               // CONN_GPIO7
	MX6Q_PAD_ENET_TXD1__GPIO_1_29,               // CONN_GPIO8
	MX6Q_PAD_SD4_DAT4__GPIO_2_12,                // CONN_GPIO9
	MX6Q_PAD_GPIO_6__GPIO_1_6,                   // CONN_GPIO10

	// I2CA
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,

	// I2CB
	MX6Q_PAD_GPIO_5__I2C3_SCL,
	MX6Q_PAD_GPIO_16__I2C3_SDA,

	// INTERRUPTS
	MX6Q_PAD_EIM_BCLK__GPIO_6_31,                // SOM_IRQA#
	MX6Q_PAD_EIM_EB0__GPIO_2_28,                 // SOM_IRQB#
	MX6Q_PAD_EIM_EB1__GPIO_2_29,                 // SOM_IRQC#

	// LCD
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,            // LCD_HSYNC
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,            // LCD_VSYNC
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,          // LCD_DE
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
	MX6Q_PAD_DI0_PIN4__GPIO_4_20,                // LCD_ENA (EN_PANL)
	MX6Q_PAD_GPIO_3__GPIO_1_3,                   // LCD_BL_ENA (EN_LITE)
	MX6Q_PAD_SD4_DAT1__PWM3_PWMO,                // LCD_PWM1
	MX6Q_PAD_SD4_DAT2__PWM4_PWMO,                // LCD_PWM2

	// MISC
	MX6Q_PAD_GPIO_0__GPIO_1_0,                   // WAKEUP#
	MX6Q_PAD_GPIO_2__GPIO_1_2,                   // WIRELESS_PWR_EN
	MX6Q_PAD_GPIO_18__GPIO_7_13,                 // LED_CNTRL
	MX6Q_PAD_SD4_DAT0__GPIO_2_8,                 // PER_RST

	// NAND
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_CS1__RAWNAND_CE1N,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,

	// SD1
	MX6Q_PAD_SD1_CMD__USDHC1_CMD_50MHZ_40OHM,
	MX6Q_PAD_SD1_CLK__USDHC1_CLK_50MHZ_40OHM,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0_50MHZ_40OHM,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1_50MHZ_40OHM,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2_50MHZ_40OHM,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3_50MHZ_40OHM,
	MX6Q_PAD_SD3_DAT4__GPIO_7_1,                 // SD1_RST
	MX6Q_PAD_GPIO_9__GPIO_1_9,                   // SD1_WP
	MX6Q_PAD_GPIO_1__GPIO_1_1,                   // SD1_CD

	// SD2
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,

	// SPI
	MX6Q_PAD_SD2_CMD__ECSPI5_MOSI,
	MX6Q_PAD_SD2_CLK__ECSPI5_SCLK,
	MX6Q_PAD_SD2_DAT0__ECSPI5_MISO,
	MX6Q_PAD_SD2_DAT2__ECSPI5_SS1,
	MX6Q_PAD_SD2_DAT3__ECSPI5_SS3,

	// UARTA
	MX6Q_PAD_GPIO_7__UART2_TXD,
	MX6Q_PAD_GPIO_8__UART2_RXD,
	MX6Q_PAD_SD4_DAT5__GPIO_2_13,                // UARTA_ENTX

	// UARTB
	MX6Q_PAD_SD3_DAT6__UART1_RXD,
	MX6Q_PAD_SD3_DAT7__UART1_TXD,

	// USB OTG
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,

};

static iomux_v3_cfg_t mx6q_bctrm3_csi0_sensor_pads[] = {
	// CAM
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_DAT4__IPU1_CSI0_D_4,
	MX6Q_PAD_CSI0_DAT5__IPU1_CSI0_D_5,
	MX6Q_PAD_CSI0_DAT6__IPU1_CSI0_D_6,
	MX6Q_PAD_CSI0_DAT7__IPU1_CSI0_D_7,
	MX6Q_PAD_CSI0_DAT8__IPU1_CSI0_D_8,
	MX6Q_PAD_CSI0_DAT9__IPU1_CSI0_D_9,
	MX6Q_PAD_CSI0_DAT10__IPU1_CSI0_D_10,
	MX6Q_PAD_CSI0_DAT11__IPU1_CSI0_D_11,
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_EIM_LBA__GPIO_2_27,                 // CAM_IRQ
	MX6Q_PAD_NANDF_CS2__GPIO_6_15,               // CAM_GLOBAL_RESET
	MX6Q_PAD_NANDF_CS3__GPIO_6_16,               // CAM_STROBE
};

static iomux_v3_cfg_t bctrm3_hdmi_ddc_pads[] = {
	MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t bctrm3_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};


#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd3_pad_change(unsigned int index, int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_200mhz,
					ARRAY_SIZE(mx6q_sd3_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_100mhz,
					ARRAY_SIZE(mx6q_sd3_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_50mhz,
					ARRAY_SIZE(mx6q_sd3_50mhz));
	}
}

static int plt_sd1_pad_change(unsigned int index, int clock)
{
	return 0;
}

static const struct esdhc_platform_data mx6q_bctrm3_sd3_data __initconst = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd3_pad_change,
};

static const struct esdhc_platform_data mx6q_bctrm3_sd1_data __initconst = {
	.cd_gpio = MX6Q_BCTRM3_SD1_CD,
	.wp_gpio = -EINVAL,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd1_pad_change,
};

static int gpmi_nand_platform_init(void)
{
	return 0;
}

static struct mtd_partition nand_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= 0x00200000,  //2M (16 blocks)
	},{
		.name	= "Env",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0x00100000,  //1M (8 blocks)
	},{
		.name	= "splash",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0xA00000,   //10MB (80 blocks)
	},{
		.name	= "kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 0xA00000,  //10MB (80 blocks)
	},{
		.name	= "rootfs",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct gpmi_nand_platform_data
mx6_gpmi_nand_platform_data = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.enable_bbt              = 0,
	.enable_ddr              = 0,
	.partitions				 = nand_partitions,
	.partition_count		 = 4,
};

static int __init board_support_onfi_nand(char *p)
{
	mx6_gpmi_nand_platform_data.enable_ddr = 1;
	return 0;
}

early_param("onfi_support", board_support_onfi_nand);


static const struct anatop_thermal_platform_data
	mx6q_bctrm3_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_bctrm3_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
}

static void phy_write_mmd(struct phy_device *phydev, u32 regnum, u16 val)
{
	phy_write(phydev, 0x0d, 0x0002);
	phy_write(phydev, 0x0e, regnum);
	phy_write(phydev, 0x0d, 0x8002);
	phy_write(phydev, 0x0e, val);
}

static int mx6q_bctrm3_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1F00);

	// Use defaults for RX, max delay for TX (i.e. delay TX_CLK as much as possible, TXD as little as possible)
	phy_write_mmd(phydev, 0x4, 0x0070);		// RX_CTL[7:4], TX_CTL[3:0]
	phy_write_mmd(phydev, 0x5, 0x7777);		// RXDn
	phy_write_mmd(phydev, 0x6, 0x0000);		// TXDn
	phy_write_mmd(phydev, 0x8, 0x03EF);		// TX_CLK[9:5], RX_CLK[4:0]

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_bctrm3_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_bctrm3_spi_cs[] = {
	MX6Q_BCTRM3_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_bctrm3_spi_data __initconst = {
	.chipselect     = mx6q_bctrm3_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_bctrm3_spi_cs),
};

/* fixed regulator for ads7846 */
static struct regulator_consumer_supply ads7846_supply =
	REGULATOR_SUPPLY("vcc", "spi4.0");

static struct regulator_init_data vads7846_regulator = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ads7846_supply,
};

static struct fixed_voltage_config vads7846 = {
	.supply_name		= "vads7846",
	.microvolts		= 3300000, /* 3.3V */
	.gpio			= -EINVAL,
	.startup_delay		= 0,
	.init_data		= &vads7846_regulator,
};

static struct platform_device vads7846_device = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev = {
		.platform_data = &vads7846,
	},
};


static int ads7846_get_pendown_state(void)
{
	//printk("ads7846_get_pendown_state: %d\n", gpio_get_value(MX6Q_BCTRM3_TCH_INT1));
	return !gpio_get_value(MX6Q_BCTRM3_TCH_INT1);
}

static struct ads7846_platform_data ads7846_data = {
	.x_max = 0x0fff,
	.y_max = 0x0fff,
	.x_plate_ohms = 180,
	.pressure_max = 255,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on = 1,
	.settle_delay_usecs	= 150,
	.wakeup				= true,
	.gpio_pendown = MX6Q_BCTRM3_TCH_INT1,
};

static struct spi_board_info imx6_bctrm3_spi_nor_device[] __initdata = {
	{
		.modalias               = "ads7846",
		.platform_data          = &ads7846_data,
		.bus_num                = 4,
		.irq                    = gpio_to_irq(MX6Q_BCTRM3_TCH_INT1),
		.chip_select            = 0,
		.max_speed_hz           = 187500,
	},
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_bctrm3_spi_nor_device,
				ARRAY_SIZE(imx6_bctrm3_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_bctrm3_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_bctrm3_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
};

static struct platform_device mx6_bctrm3_audio_device = {
	.name = "imx-bctrm3",
};

static struct imxi2c_platform_data mx6q_bctrm3_i2c_data = {
	.bitrate = 400000,
};


static void mx6q_csi0_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_bctrm3_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_bctrm3_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(MX6Q_BCTRM3_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6Q_BCTRM3_CSI0_RST, 1);

	gpio_set_value(MX6Q_BCTRM3_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(MX6Q_BCTRM3_CSI0_RST, 1);

	/* For MX6Q GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 */
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
};



static void imx6q_bctrm3_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6Q_BCTRM3_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_BCTRM3_USB_OTG_PWR, 0);
}

static void __init imx6q_bctrm3_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_BCTRM3_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_BCTRM3_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_BCTRM3_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_bctrm3_usbotg_vbus);
//	mx6_usb_dr_init();
//	mx6_usb_h1_init();
}

static struct aic3x_pdata bctrm3_aic33_data __initdata = {
	.gpio_reset = -EINVAL,
};

// I2CA has HDMI DDC
static struct i2c_board_info mxc_i2c1_board_info[] __initdata =
{
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
#ifdef CONFIG_BCT_USE_HB_CODEC
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
		.platform_data = (void *)&bctrm3_aic33_data,
	},
#endif
};

// I2CB has SoM codec
static struct i2c_board_info mxc_i2c2_board_info[] __initdata =
{
#ifndef CONFIG_BCT_USE_HB_CODEC
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
		.platform_data = (void *)&bctrm3_aic33_data,
	},
#endif
};



/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_bctrm3_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_bctrm3_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_bctrm3_sata_data = {
	.init = mx6q_bctrm3_sata_init,
	.exit = mx6q_bctrm3_sata_exit,
};

static struct gpio mx6q_bctrm3_flexcan_gpios[] = {
	{ MX6Q_BCTRM3_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
};

static void mx6q_bctrm3_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6Q_BCTRM3_CAN1_EN, 1);
	} else {
		gpio_set_value(MX6Q_BCTRM3_CAN1_EN, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_bctrm3_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_bctrm3_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data bctrm3_fb_data[] = {
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "LDB-XGA",
	.default_bpp = 32,
	.int_clk = false,
	},
	{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "XGA",
	.default_bpp = 32,
	.int_clk = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if ((mx6q_revision() > IMX_CHIP_REVISION_1_1))
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(bctrm3_hdmi_ddc_pads,
		ARRAY_SIZE(bctrm3_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(bctrm3_i2c2_pads,
		ARRAY_SIZE(bctrm3_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB32,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	}, {
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void bctrm3_suspend_enter(void)
{
	/* suspend preparation */
}

static void bctrm3_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_bctrm3_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = bctrm3_suspend_enter,
	.suspend_exit = bctrm3_suspend_exit,
};

static struct regulator_consumer_supply bctrm3_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data bctrm3_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(bctrm3_vmmc_consumers),
	.consumer_supplies = bctrm3_vmmc_consumers,
};

static struct fixed_voltage_config bctrm3_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &bctrm3_vmmc_init,
};

static struct platform_device bctrm3_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &bctrm3_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_bctrm3_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_bctrm3_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_bctrm3_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_bctrm3_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_bctrm3_consumer_vdda,
};

static struct regulator_init_data sgtl5000_bctrm3_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_bctrm3_consumer_vddio,
};

static struct regulator_init_data sgtl5000_bctrm3_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_bctrm3_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_bctrm3_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 2500000,
	.gpio			= -1,
	.init_data		= &sgtl5000_bctrm3_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_bctrm3_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_bctrm3_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_bctrm3_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_bctrm3_vddd_reg_initdata,
};

static struct platform_device sgtl5000_bctrm3_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_bctrm3_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_bctrm3_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_bctrm3_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_bctrm3_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_bctrm3_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_bctrm3_audio_device,
			    &mx6_bctrm3_audio_data);
	imx6q_add_imx_ssi(1, &mx6_bctrm3_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_bctrm3_vdda_reg_devices);
	platform_device_register(&sgtl5000_bctrm3_vddio_reg_devices);
	platform_device_register(&sgtl5000_bctrm3_vddd_reg_devices);
#endif
	return 0;
}

static struct platform_pwm_backlight_data mx6_bctrm3_pwm_backlight_data = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data bctrm3_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	printk(KERN_ERR "fixup_mxc_board\n");
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};


static void __init bcthb2_ads7846_init(void)
{
	if (gpio_request_one(MX6Q_BCTRM3_TCH_INT1, GPIOF_IN, "ADS7846 pendown"))
	{
		printk("Failed to request ADS7846 pendown GPIO\n");
		return;
	}

	return;
}


void bctrm3_pcie_reset(void)
{
	struct i2c_adapter *adapter = i2c_get_adapter(2);
	struct i2c_msg msg;
	int status;

	char data[] = {0x0A ,0x00};

	if (!adapter)
	{
		printk(KERN_ERR "bctrm3_pcie_reset: Missing I2C adapter - cannot reset PCIE card\n");
		return;
	}

	msg.addr = 0x56;
	msg.buf = (void __force *) data;
	msg.len = 2;
	msg.flags = 0;

	status = i2c_transfer(adapter, &msg, 1);

	if(status < 0)
	{
		printk(KERN_ERR "bctrm3_pcie_reset: I2C transfer to PIC failed: %d\n", status);
	}
	else
	{
		// Wait for PIC to perform the reset
		msleep(100);
		printk(KERN_INFO "Done BCTRM3 PCIE Reset OK\n");
	}
}

static const struct imx_pcie_platform_data bctrm3_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= -EINVAL,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
	.reset_func	= bctrm3_pcie_reset,
};


/*!
 * Board specific initialization.
 */
static void __init mx6_bctrm3_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	mxc_iomux_v3_setup_multiple_pads(mx6q_bctrm3_pads,
					ARRAY_SIZE(mx6q_bctrm3_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = bctrm3_dvfscore_data.reg_id;
	mx6q_bctrm3_init_uart();
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(bctrm3_fb_data); i++)
	{
		imx6q_add_ipuv3fb(i, &bctrm3_fb_data[i]);
	}

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	gpio_request(MX6Q_BCTRM3_AUDIO_SELECT, "AUDIO_SELECT");
#ifdef CONFIG_BCT_USE_HB_CODEC
	gpio_direction_output(MX6Q_BCTRM3_AUDIO_SELECT, 1);		// Default to HB codec
#else
	gpio_direction_output(MX6Q_BCTRM3_AUDIO_SELECT, 0);		// Default to RM3 codec
#endif
	gpio_export(MX6Q_BCTRM3_AUDIO_SELECT, 0);

	gpio_request(MX6Q_BCTRM3_EN_LITE, "EN_BL");
	gpio_direction_output(MX6Q_BCTRM3_EN_LITE, 1);
	gpio_export(MX6Q_BCTRM3_EN_LITE, 0);

	gpio_request(MX6Q_BCTRM3_EN_PANEL, "EN_PANEL");
	gpio_direction_output(MX6Q_BCTRM3_EN_PANEL, 1);
	gpio_export(MX6Q_BCTRM3_EN_PANEL, 0);

	//request GPIO's - default to 4 in + 4 out, but can be altered by user
	gpio_request(MX6Q_BCTRM3_GPIO1, "RM3_GPIO1");
	gpio_direction_output(MX6Q_BCTRM3_GPIO1, 1);
	gpio_export(MX6Q_BCTRM3_GPIO1, 1);

	gpio_request(MX6Q_BCTRM3_GPIO2, "RM3_GPIO2");
	gpio_direction_output(MX6Q_BCTRM3_GPIO2, 1);
	gpio_export(MX6Q_BCTRM3_GPIO2, 1);

	gpio_request(MX6Q_BCTRM3_GPIO3, "RM3_GPIO3");
	gpio_direction_output(MX6Q_BCTRM3_GPIO3, 1);
	gpio_export(MX6Q_BCTRM3_GPIO3, 1);

	gpio_request(MX6Q_BCTRM3_GPIO4, "RM3_GPIO4");
	gpio_direction_output(MX6Q_BCTRM3_GPIO4, 1);
	gpio_export(MX6Q_BCTRM3_GPIO4, 1);

	gpio_request(MX6Q_BCTRM3_GPIO5, "RM3_GPIO5");
	gpio_direction_input(MX6Q_BCTRM3_GPIO5);
	gpio_export(MX6Q_BCTRM3_GPIO5, 1);

	gpio_request(MX6Q_BCTRM3_GPIO6, "RM3_GPIO6");
	gpio_direction_input(MX6Q_BCTRM3_GPIO6);
	gpio_export(MX6Q_BCTRM3_GPIO6, 1);

	gpio_request(MX6Q_BCTRM3_GPIO7, "RM3_GPIO7");
	gpio_direction_input(MX6Q_BCTRM3_GPIO7);
	gpio_export(MX6Q_BCTRM3_GPIO7, 1);

	gpio_request(MX6Q_BCTRM3_GPIO8, "RM3_GPIO8");
	gpio_direction_input(MX6Q_BCTRM3_GPIO8);
	gpio_export(MX6Q_BCTRM3_GPIO8, 1);

	gpio_request(MX6Q_BCTRM3_GPIO9, "RM3_GPIO9");
#ifdef CONFIG_BCT_USE_HB_CODEC
	gpio_direction_output(MX6Q_BCTRM3_GPIO9, 1);	// Enable IIS signals to HB codec
#else
	gpio_direction_output(MX6Q_BCTRM3_GPIO9, 0);
#endif
	gpio_export(MX6Q_BCTRM3_GPIO9, 1);

	gpio_request(MX6Q_BCTRM3_WIRELESS_PWR_EN, "WIRELESS_PWR_EN");
	gpio_direction_output(MX6Q_BCTRM3_WIRELESS_PWR_EN, 0);
	gpio_export(MX6Q_BCTRM3_WIRELESS_PWR_EN, 0);

	gpio_request(MX6Q_BCTRM3_PER_RST, "PER_RST");
	gpio_direction_output(MX6Q_BCTRM3_PER_RST, 1);
	msleep(20);
	gpio_set_value(MX6Q_BCTRM3_PER_RST, 0);

	bctrm3_init_smsc911x();

	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info, ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_imx_i2c(0, &mx6q_bctrm3_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_bctrm3_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_bctrm3_i2c_data);

	/* SPI */
	platform_device_register(&vads7846_device);
	imx6q_add_ecspi(4, &mx6q_bctrm3_spi_data);
	bcthb2_ads7846_init();
	spi_device_init();


	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_bctrm3_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_bctrm3_pm_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_bctrm3_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_bctrm3_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_bctrm3_init_usb();
	imx6q_add_ahci(0, &mx6q_bctrm3_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&bctrm3_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(2, &mx6_bctrm3_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&bctrm3_dvfscore_data);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	ret = gpio_request_array(mx6q_bctrm3_flexcan_gpios,
			ARRAY_SIZE(mx6q_bctrm3_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_bctrm3_flexcan0_pdata);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	imx6q_add_busfreq();

	imx6q_add_pcie(&bctrm3_pcie_data);
}

extern void __iomem *twd_base;
static void __init mx6_bctrm3_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_bctrm3_timer = {
	.init   = mx6_bctrm3_timer_init,
};

static void __init mx6q_bctrm3_reserve(void)
{
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		//memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABRELITE data structure.
 */
MACHINE_START(BCTRM3, "BCT i.MX 6Quad RM3 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_bctrm3_board_init,
	.timer = &mx6_bctrm3_timer,
	.reserve = mx6q_bctrm3_reserve,
MACHINE_END
