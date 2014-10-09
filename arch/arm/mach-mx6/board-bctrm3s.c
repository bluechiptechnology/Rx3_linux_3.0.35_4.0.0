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
#define RM3S_GPIO(num) {\
	gpio_request(num, "RM3S_GPIO_" #num);\
	gpio_direction_input(num);\
	gpio_export(num, 1);\
	}

#define MX6Q_BCTRM3_SD1_CD          IMX_GPIO_NR(1, 1)
#define MX6Q_BCTRM3_SD1_WP          IMX_GPIO_NR(1, 9)
#define MX6Q_BCTRM3_SD2_CD          IMX_GPIO_NR(2, 23)
#define MX6Q_BCTRM3_SD2_WP          IMX_GPIO_NR(2, 24)
#define MX6Q_BCTRM3_ECSPI1_CS1      IMX_GPIO_NR(3, 19)
#define MX6Q_BCTRM3_ECSPI5_CS1      IMX_GPIO_NR(1, 13)
#define MX6Q_BCTRM3_ECSPI5_CS3      IMX_GPIO_NR(1, 12)
#define MX6Q_BCTRM3_ECSPI5_MOSI     IMX_GPIO_NR(1, 11)
#define MX6Q_BCTRM3_ECSPI5_MISO     IMX_GPIO_NR(1, 15)
#define MX6Q_BCTRM3_ECSPI5_SCLK     IMX_GPIO_NR(1, 10)

#define MX6Q_BCTRM3_CAN_1           IMX_GPIO_NR(4, 15)
#define MX6Q_BCTRM3_CAN_2           IMX_GPIO_NR(4, 14)
#define MX6Q_BCTRM3_CAN_3           IMX_GPIO_NR(2, 11)
#define MX6Q_BCTRM3_LED_CNTRL       IMX_GPIO_NR(7, 13)

#define MX6Q_BCTRM3_UARTA_TX        IMX_GPIO_NR(1, 7)
#define MX6Q_BCTRM3_UARTA_RX        IMX_GPIO_NR(1, 8)
#define MX6Q_BCTRM3_UARTA_ENTX      IMX_GPIO_NR(2, 13)
#define MX6Q_BCTRM3_UARTA_CTS       IMX_GPIO_NR(3, 28)
#define MX6Q_BCTRM3_UARTA_RTS       IMX_GPIO_NR(3, 29)
#define MX6Q_BCTRM3_UARTB_CTS       IMX_GPIO_NR(3, 30)
#define MX6Q_BCTRM3_UARTB_RTS       IMX_GPIO_NR(3, 31)

#define MX6Q_BCTRM3_CAM_HS          IMX_GPIO_NR(5, 19)
#define MX6Q_BCTRM3_CAM_PCLK        IMX_GPIO_NR(5, 18)
#define MX6Q_BCTRM3_CAM_VS          IMX_GPIO_NR(5, 21)
#define MX6Q_BCTRM3_CAM_WEN         IMX_GPIO_NR(5, 20)
#define MX6Q_BCTRM3_CAM_XD0         IMX_GPIO_NR(5, 22)
#define MX6Q_BCTRM3_CAM_XD1         IMX_GPIO_NR(5, 23)
#define MX6Q_BCTRM3_CAM_XD2         IMX_GPIO_NR(5, 24)
#define MX6Q_BCTRM3_CAM_XD3         IMX_GPIO_NR(5, 25)
#define MX6Q_BCTRM3_CAM_XD4         IMX_GPIO_NR(5, 26)
#define MX6Q_BCTRM3_CAM_XD5         IMX_GPIO_NR(5, 27)
#define MX6Q_BCTRM3_CAM_D0          IMX_GPIO_NR(5, 28)
#define MX6Q_BCTRM3_CAM_D1          IMX_GPIO_NR(5, 29)
#define MX6Q_BCTRM3_CAM_D2          IMX_GPIO_NR(5, 30)
#define MX6Q_BCTRM3_CAM_D3          IMX_GPIO_NR(5, 31)
#define MX6Q_BCTRM3_CAM_D4          IMX_GPIO_NR(6, 0)
#define MX6Q_BCTRM3_CAM_D5          IMX_GPIO_NR(6, 1)
#define MX6Q_BCTRM3_CAM_D6          IMX_GPIO_NR(6, 2)
#define MX6Q_BCTRM3_CAM_D7          IMX_GPIO_NR(6, 3)
#define MX6Q_BCTRM3_CAM_D8          IMX_GPIO_NR(6, 4)
#define MX6Q_BCTRM3_CAM_D9          IMX_GPIO_NR(6, 5)
#define MX6Q_BCTRM3_CAM_IRQ         IMX_GPIO_NR(2, 27)
#define MX6Q_BCTRM3_CAM_GLOBAL_RST  IMX_GPIO_NR(6, 15)
#define MX6Q_BCTRM3_CAM_STROBE      IMX_GPIO_NR(6, 16)

#define MX6Q_BCTRM3_AUDIO_SELECT    IMX_GPIO_NR(2, 14)

#define MX6Q_BCTRM3_GPIO1           IMX_GPIO_NR(1, 4)
#define MX6Q_BCTRM3_GPIO2           IMX_GPIO_NR(4, 10)
#define MX6Q_BCTRM3_GPIO3           IMX_GPIO_NR(1, 25)
#define MX6Q_BCTRM3_GPIO4           IMX_GPIO_NR(1, 28)
#define MX6Q_BCTRM3_GPIO5           IMX_GPIO_NR(1, 27)
#define MX6Q_BCTRM3_GPIO6           IMX_GPIO_NR(1, 26)
#define MX6Q_BCTRM3_GPIO7           IMX_GPIO_NR(1, 30)
#define MX6Q_BCTRM3_GPIO8           IMX_GPIO_NR(1, 29)
#define MX6Q_BCTRM3_GPIO9           IMX_GPIO_NR(2, 12)
#define MX6Q_BCTRM3_GPIO10          IMX_GPIO_NR(3, 11)
#define MX6Q_BCTRM3_GPIO11          IMX_GPIO_NR(3, 21)
#define MX6Q_BCTRM3_GPIO12          IMX_GPIO_NR(3, 22)
#define MX6Q_BCTRM3_GPIO13          IMX_GPIO_NR(3, 23)
#define MX6Q_BCTRM3_GPIO14          IMX_GPIO_NR(3, 26)
#define MX6Q_BCTRM3_GPIO15          IMX_GPIO_NR(3, 27)

#define MX6Q_BCTRM3_LCD_PCLK        IMX_GPIO_NR(4, 16)
#define MX6Q_BCTRM3_LCD_HSYNC       IMX_GPIO_NR(4, 18)
#define MX6Q_BCTRM3_LCD_VSYNC       IMX_GPIO_NR(4, 19)
#define MX6Q_BCTRM3_LCD_DE          IMX_GPIO_NR(4, 17)
#define MX6Q_BCTRM3_LCD_D0          IMX_GPIO_NR(4, 21)
#define MX6Q_BCTRM3_LCD_D1          IMX_GPIO_NR(4, 22)
#define MX6Q_BCTRM3_LCD_D2          IMX_GPIO_NR(4, 23)
#define MX6Q_BCTRM3_LCD_D3          IMX_GPIO_NR(4, 24)
#define MX6Q_BCTRM3_LCD_D4          IMX_GPIO_NR(4, 25)
#define MX6Q_BCTRM3_LCD_D5          IMX_GPIO_NR(4, 26)
#define MX6Q_BCTRM3_LCD_D6          IMX_GPIO_NR(4, 27)
#define MX6Q_BCTRM3_LCD_D7          IMX_GPIO_NR(4, 28)
#define MX6Q_BCTRM3_LCD_D8          IMX_GPIO_NR(4, 29)
#define MX6Q_BCTRM3_LCD_D9          IMX_GPIO_NR(4, 30)
#define MX6Q_BCTRM3_LCD_D10         IMX_GPIO_NR(4, 31)
#define MX6Q_BCTRM3_LCD_D11         IMX_GPIO_NR(5, 5)
#define MX6Q_BCTRM3_LCD_D12         IMX_GPIO_NR(5, 6)
#define MX6Q_BCTRM3_LCD_D13         IMX_GPIO_NR(5, 7)
#define MX6Q_BCTRM3_LCD_D14         IMX_GPIO_NR(5, 8)
#define MX6Q_BCTRM3_LCD_D15         IMX_GPIO_NR(5, 9)
#define MX6Q_BCTRM3_LCD_D16         IMX_GPIO_NR(5, 10)
#define MX6Q_BCTRM3_LCD_D17         IMX_GPIO_NR(5, 11)
#define MX6Q_BCTRM3_LCD_D18         IMX_GPIO_NR(5, 12)
#define MX6Q_BCTRM3_LCD_D19         IMX_GPIO_NR(5, 13)
#define MX6Q_BCTRM3_LCD_D20         IMX_GPIO_NR(5, 14)
#define MX6Q_BCTRM3_LCD_D21         IMX_GPIO_NR(5, 15)
#define MX6Q_BCTRM3_LCD_D22         IMX_GPIO_NR(5, 16)
#define MX6Q_BCTRM3_LCD_D23         IMX_GPIO_NR(5, 17)
#define MX6Q_BCTRM3_EN_PANEL        IMX_GPIO_NR(4, 20)     // LCD_ENA
#define MX6Q_BCTRM3_EN_LITE         IMX_GPIO_NR(1, 3)      // LCD_BL_ENA
#define MX6Q_BCTRM3_LCD_PWM1        IMX_GPIO_NR(2, 9)
#define MX6Q_BCTRM3_LCD_PWM2        IMX_GPIO_NR(2, 10)

#define MX6Q_BCTRM3_CEC             IMX_GPIO_NR(4, 11)
#define MX6Q_BCTRM3_WAKEUP          IMX_GPIO_NR(2, 25)
#define MX6Q_BCTRM3_OTG_ID          IMX_GPIO_NR(1, 24)
#define MX6Q_BCTRM3_PER_RST         IMX_GPIO_NR(2, 8)
#define MX6Q_BCTRM3_WIRELESS_PWR_EN IMX_GPIO_NR(1, 2)

#define MX6Q_BCTRM3_SOM_IRQA_GPIO   IMX_GPIO_NR(6, 31)
#define MX6Q_BCTRM3_SOM_IRQB_GPIO   IMX_GPIO_NR(2, 28)
#define MX6Q_BCTRM3_SOM_IRQC_GPIO   IMX_GPIO_NR(2, 29)

//
// GPIO mappings
//
#define MX6Q_BCTRM3_SPI_CS_FLASH    MX6Q_BCTRM3_ECSPI1_CS1


void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern char *gp_reg_id;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);

static iomux_v3_cfg_t mx6q_bctrm3_pads[] = {

	// AUDIO (IIS)
	MX6Q_PAD_KEY_COL0__AUDMUX_AUD5_TXC,          // IIS_CLKX
	MX6Q_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,          // IIS_DX
	MX6Q_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,         // IIS_FSX
	MX6Q_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,          // IIS_DR
	MX6Q_PAD_SD4_DAT6__GPIO_2_14,                // AUDIO_SELECT

	// CAN
	MX6Q_PAD_KEY_COL4__GPIO_4_14,
	MX6Q_PAD_KEY_ROW4__GPIO_4_15,
	MX6Q_PAD_SD4_DAT3__GPIO_2_11,                // CAN_3

	// ECSPI1 (SPI NOR Flash)
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D19__GPIO_3_19,                 // CS1

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
	MX6Q_PAD_EIM_DA11__GPIO_3_11,                // CONN_GPIO10
	MX6Q_PAD_EIM_D21__GPIO_3_21,                 // CONN_GPIO11
	MX6Q_PAD_EIM_D22__GPIO_3_22,                 // CONN_GPIO12
	MX6Q_PAD_EIM_D23__GPIO_3_23,                 // CONN_GPIO13
	MX6Q_PAD_EIM_D26__GPIO_3_26,                 // CONN_GPIO14
	MX6Q_PAD_EIM_D27__GPIO_3_27,                 // CONN_GPIO15

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
	MX6Q_PAD_DI0_DISP_CLK__GPIO_4_16,            // LCD PCLK
	MX6Q_PAD_DI0_PIN2__GPIO_4_18,                // LCD_HSYNC
	MX6Q_PAD_DI0_PIN3__GPIO_4_19,                // LCD_VSYNC
	MX6Q_PAD_DI0_PIN15__GPIO_4_17,               // LCD_DE
	MX6Q_PAD_DISP0_DAT0__GPIO_4_21,
	MX6Q_PAD_DISP0_DAT1__GPIO_4_22,
	MX6Q_PAD_DISP0_DAT2__GPIO_4_23,
	MX6Q_PAD_DISP0_DAT3__GPIO_4_24,
	MX6Q_PAD_DISP0_DAT4__GPIO_4_25,
	MX6Q_PAD_DISP0_DAT5__GPIO_4_26,
	MX6Q_PAD_DISP0_DAT6__GPIO_4_27,
	MX6Q_PAD_DISP0_DAT7__GPIO_4_28,
	MX6Q_PAD_DISP0_DAT8__GPIO_4_29,
	MX6Q_PAD_DISP0_DAT9__GPIO_4_30,
	MX6Q_PAD_DISP0_DAT10__GPIO_4_31,
	MX6Q_PAD_DISP0_DAT11__GPIO_5_5,
	MX6Q_PAD_DISP0_DAT12__GPIO_5_6,
	MX6Q_PAD_DISP0_DAT13__GPIO_5_7,
	MX6Q_PAD_DISP0_DAT14__GPIO_5_8,
	MX6Q_PAD_DISP0_DAT15__GPIO_5_9,
	MX6Q_PAD_DISP0_DAT16__GPIO_5_10,
	MX6Q_PAD_DISP0_DAT17__GPIO_5_11,
	MX6Q_PAD_DISP0_DAT18__GPIO_5_12,
	MX6Q_PAD_DISP0_DAT19__GPIO_5_13,
	MX6Q_PAD_DISP0_DAT20__GPIO_5_14,
	MX6Q_PAD_DISP0_DAT21__GPIO_5_15,
	MX6Q_PAD_DISP0_DAT22__GPIO_5_16,
	MX6Q_PAD_DISP0_DAT23__GPIO_5_17,
	MX6Q_PAD_DI0_PIN4__GPIO_4_20,                // LCD_ENA (EN_PANL)
	MX6Q_PAD_GPIO_3__GPIO_1_3,                   // LCD_BL_ENA (EN_LITE)
	MX6Q_PAD_SD4_DAT1__GPIO_2_9,                 // LCD_PWM1
	MX6Q_PAD_SD4_DAT2__GPIO_2_10,                // LCD_PWM2

	// MISC
	MX6Q_PAD_EIM_OE__GPIO_2_25,                  // WAKEUP#
	MX6Q_PAD_GPIO_2__GPIO_1_2,                   // WIRELESS_PWR_EN
	MX6Q_PAD_GPIO_18__GPIO_7_13,                 // LED_CNTRL
	MX6Q_PAD_SD4_DAT0__GPIO_2_8,                 // PER_RST
	MX6Q_PAD_KEY_ROW2__GPIO_4_11,                // HDMI_CEC

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
	MX6Q_PAD_SD2_DAT1__GPIO_1_14,                // SD1_RST
	MX6Q_PAD_GPIO_9__GPIO_1_9,                   // SD1_WP
	MX6Q_PAD_GPIO_1__GPIO_1_1,                   // SD1_CD

	// SD2
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,
	MX6Q_PAD_EIM_CS0__GPIO_2_23,                 // SD2_CD
	MX6Q_PAD_EIM_CS1__GPIO_2_24,                 // SD2_WP

	// SPI
	MX6Q_PAD_SD2_CMD__GPIO_1_11,                 // SPI_SIMO
	MX6Q_PAD_SD2_CLK__GPIO_1_10,                 // SPI_CLK
	MX6Q_PAD_SD2_DAT0__GPIO_1_15,                // SPI_SOMI
	MX6Q_PAD_SD2_DAT2__GPIO_1_13,                // SPI_CSA
	MX6Q_PAD_SD2_DAT3__GPIO_1_12,                // SPI_CSB

	// UARTA
	MX6Q_PAD_GPIO_7__GPIO_1_7,                   // UARTA_TX
	MX6Q_PAD_GPIO_8__GPIO_1_8,                   // UARTA_RX
	MX6Q_PAD_SD4_DAT5__GPIO_2_13,                // UARTA_ENTX
	MX6Q_PAD_EIM_D28__GPIO_3_28,                 // UARTA_CTS
	MX6Q_PAD_EIM_D29__GPIO_3_29,                 // UARTA_RTS

	// UARTB
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,
	MX6Q_PAD_EIM_D30__GPIO_3_30,                 // UARTB_CTS
	MX6Q_PAD_EIM_D31__GPIO_3_31,                 // UARTB_RTS

	// USB OTG
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,              // USB_OTG_ID

	// CAM
	MX6Q_PAD_CSI0_MCLK__GPIO_5_19,               // CAM_HS
	MX6Q_PAD_CSI0_PIXCLK__GPIO_5_18,             // CAM_PCLK
	MX6Q_PAD_CSI0_VSYNC__GPIO_5_21,              // CAM_VS
	MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20,            // CAM_WEN
	MX6Q_PAD_CSI0_DAT4__GPIO_5_22,               // CAM_XD0
	MX6Q_PAD_CSI0_DAT5__GPIO_5_23,               // CAM_XD1
	MX6Q_PAD_CSI0_DAT6__GPIO_5_24,               // CAM_XD2
	MX6Q_PAD_CSI0_DAT7__GPIO_5_25,               // CAM_XD3
	MX6Q_PAD_CSI0_DAT8__GPIO_5_26,               // CAM_XD4
	MX6Q_PAD_CSI0_DAT9__GPIO_5_27,               // CAM_XD5
	MX6Q_PAD_CSI0_DAT10__GPIO_5_28,              // CAM_D0
	MX6Q_PAD_CSI0_DAT11__GPIO_5_29,              // CAM_D1
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,              // CAM_D2
	MX6Q_PAD_CSI0_DAT13__GPIO_5_31,              // CAM_D3
	MX6Q_PAD_CSI0_DAT14__GPIO_6_0,               // CAM_D4
	MX6Q_PAD_CSI0_DAT15__GPIO_6_1,               // CAM_D5
	MX6Q_PAD_CSI0_DAT16__GPIO_6_2,               // CAM_D6
	MX6Q_PAD_CSI0_DAT17__GPIO_6_3,               // CAM_D7
	MX6Q_PAD_CSI0_DAT18__GPIO_6_4,               // CAM_D8
	MX6Q_PAD_CSI0_DAT19__GPIO_6_5,               // CAM_D9
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
	MX6Q_PAD_SD##id##_DAT4__USDHC##id##_DAT4_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT5__USDHC##id##_DAT5_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT6__USDHC##id##_DAT6_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT7__USDHC##id##_DAT7_##speed##MHZ,	\
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
	.support_8bit = 1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd3_pad_change,
};

static const struct esdhc_platform_data mx6q_bctrm3_sd1_data __initconst = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd1_pad_change,
};

static int gpmi_nand_platform_init(void)
{
	return 0;
}

static struct mtd_partition nand_partitions[] = {
	{
		.name	= "lower1GB",
		.offset	= 0,
		.size	= 0x40000000,  // 1GB
	},{
		.name	= "upper1GB",
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
	.partition_count		 = ARRAY_SIZE(nand_partitions),
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
	imx6q_add_imx_uart(2, NULL);	// UART3
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

#ifdef CONFIG_MTD_M25P80
static int mx6q_bctrm3_ecspi1_cs[] = {
	MX6Q_BCTRM3_SPI_CS_FLASH,
};

static const struct spi_imx_master mx6q_bctrm3_ecspi1_data __initconst = {
	.chipselect     = mx6q_bctrm3_ecspi1_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_bctrm3_ecspi1_cs),
};
#endif

#if defined(CONFIG_MTD_M25P80)
static struct mtd_partition imx6_bctrm3_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00C0000,		// 768KBytes
	},
	{
	 .name = "bootenv",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_bctrm3_spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_bctrm3_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_bctrm3_spi_nor_partitions),
	.type = "sst25vf080b",
};
#endif

static struct spi_board_info imx6_bctrm3_spi_devices[] __initdata = {
#ifdef CONFIG_MTD_M25P80
	{
		.modalias               = "m25p80",
		.max_speed_hz           = 20000000,
		.bus_num                = 0,
		.chip_select            = 0,  	// This is an index into mx6q_bctrm3_ecspi1_cs[], NOT a hw CS number
		.platform_data          = &imx6_bctrm3_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_bctrm3_spi_devices,
				ARRAY_SIZE(imx6_bctrm3_spi_devices));
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
	.bitrate = 375000,	// See erratum ERR007805
};

static void imx6q_bctrm3_usbotg_vbus(bool on) { return; }

static void __init imx6q_bctrm3_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_bctrm3_usbotg_vbus);
//	mx6_usb_dr_init();
//	mx6_usb_h1_init();
}

static void mx6q_bctrm3_start_csi_mclk(void)
{
	struct clk *clk;
	uint32_t freq = 0;
	clk = clk_get(NULL, "csi_mclk1");
	freq = clk_round_rate(clk, 15000000);
	clk_set_rate(clk, freq);
	clk_enable(clk);
	clk_put(clk);
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
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "LDB-XGA",
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
	.ipu_id = 0,
	.disp_id = 1,
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


/*!
 * Board specific initialization.
 */
static void __init mx6_bctrm3_board_init(void)
{
	int i;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	mxc_iomux_v3_setup_multiple_pads(mx6q_bctrm3_pads,
					ARRAY_SIZE(mx6q_bctrm3_pads));

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
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	RM3S_GPIO(MX6Q_BCTRM3_CAM_HS);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_PCLK);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_VS);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_WEN);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD0);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD1);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD2);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD3);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD4);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_XD5);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D0);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D1);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D2);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D3);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D4);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D5);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D6);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D7);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D8);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_D9);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_IRQ);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_GLOBAL_RST);
	RM3S_GPIO(MX6Q_BCTRM3_CAM_STROBE);

	gpio_request(MX6Q_BCTRM3_AUDIO_SELECT, "AUDIO_SELECT");
	gpio_direction_output(MX6Q_BCTRM3_AUDIO_SELECT, 0);		// Default to RM3 codec
	gpio_export(MX6Q_BCTRM3_AUDIO_SELECT, 0);

	RM3S_GPIO(MX6Q_BCTRM3_LCD_PCLK);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_HSYNC);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_VSYNC);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_DE);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D0);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D1);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D2);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D3);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D4);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D5);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D6);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D7);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D8);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D9);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D10);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D11);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D12);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D13);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D14);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D15);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D16);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D17);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D18);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D19);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D20);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D21);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D22);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_D23);
	RM3S_GPIO(MX6Q_BCTRM3_EN_PANEL);
	RM3S_GPIO(MX6Q_BCTRM3_EN_LITE);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_PWM1);
	RM3S_GPIO(MX6Q_BCTRM3_LCD_PWM2);

	RM3S_GPIO(MX6Q_BCTRM3_GPIO1);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO2);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO3);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO4);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO5);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO6);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO7);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO8);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO9);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO10);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO11);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO12);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO13);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO14);
	RM3S_GPIO(MX6Q_BCTRM3_GPIO15);
	
	RM3S_GPIO(MX6Q_BCTRM3_WIRELESS_PWR_EN);
	RM3S_GPIO(MX6Q_BCTRM3_CEC);
	RM3S_GPIO(MX6Q_BCTRM3_WAKEUP);
	RM3S_GPIO(MX6Q_BCTRM3_OTG_ID);

	RM3S_GPIO(MX6Q_BCTRM3_CAN_1);
	RM3S_GPIO(MX6Q_BCTRM3_CAN_2);
	RM3S_GPIO(MX6Q_BCTRM3_CAN_3);

	RM3S_GPIO(MX6Q_BCTRM3_UARTA_TX);
	RM3S_GPIO(MX6Q_BCTRM3_UARTA_RX);
	RM3S_GPIO(MX6Q_BCTRM3_UARTA_ENTX);
	RM3S_GPIO(MX6Q_BCTRM3_UARTA_CTS);
	RM3S_GPIO(MX6Q_BCTRM3_UARTA_RTS);

	RM3S_GPIO(MX6Q_BCTRM3_UARTB_CTS);
	RM3S_GPIO(MX6Q_BCTRM3_UARTB_RTS);

	gpio_request(MX6Q_BCTRM3_PER_RST, "PER_RST");
	gpio_direction_output(MX6Q_BCTRM3_PER_RST, 1);
	msleep(20);
	gpio_set_value(MX6Q_BCTRM3_PER_RST, 0);

	RM3S_GPIO(MX6Q_BCTRM3_SOM_IRQA_GPIO);
	RM3S_GPIO(MX6Q_BCTRM3_SOM_IRQB_GPIO);
	RM3S_GPIO(MX6Q_BCTRM3_SOM_IRQC_GPIO);
	
	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info, ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_imx_i2c(0, &mx6q_bctrm3_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_bctrm3_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_bctrm3_i2c_data);

	/* SPI */
#ifdef CONFIG_MTD_M25P80
	imx6q_add_ecspi(0, &mx6q_bctrm3_ecspi1_data);
#endif
	spi_device_init();
	
	RM3S_GPIO(MX6Q_BCTRM3_ECSPI5_CS1);
	RM3S_GPIO(MX6Q_BCTRM3_ECSPI5_CS3);
	RM3S_GPIO(MX6Q_BCTRM3_ECSPI5_MISO);
	RM3S_GPIO(MX6Q_BCTRM3_ECSPI5_MOSI);
	RM3S_GPIO(MX6Q_BCTRM3_ECSPI5_SCLK);

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_bctrm3_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_bctrm3_pm_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_bctrm3_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_bctrm3_sd3_data);
	
	RM3S_GPIO(MX6Q_BCTRM3_SD1_CD);
	RM3S_GPIO(MX6Q_BCTRM3_SD1_WP);
	RM3S_GPIO(MX6Q_BCTRM3_SD2_CD);
	RM3S_GPIO(MX6Q_BCTRM3_SD2_WP);
	
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

	mx6q_bctrm3_start_csi_mclk();
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

	uart_clk = clk_get_sys("imx-uart.2", NULL);
	early_console_setup(UART3_BASE_ADDR, uart_clk);
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
MACHINE_START(BCTRM3S, "BCT i.MX 6Quad RM3S Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_bctrm3_board_init,
	.timer = &mx6_bctrm3_timer,
	.reserve = mx6q_bctrm3_reserve,
MACHINE_END
