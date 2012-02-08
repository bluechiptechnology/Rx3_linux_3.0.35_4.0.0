/*
 * Copyright 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ARCH_ARM_MACH_MX6_CRM_REGS_H__
#define __ARCH_ARM_MACH_MX6_CRM_REGS_H__

/* IOMUXC */
#define MXC_IOMUXC_BASE			MX6_IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR)
#define IOMUXC_GPR0			(MXC_IOMUXC_BASE + 0x00)
#define IOMUXC_GPR1			(MXC_IOMUXC_BASE + 0x04)
#define IOMUXC_GPR2			(MXC_IOMUXC_BASE + 0x08)
#define IOMUXC_GPR3			(MXC_IOMUXC_BASE + 0x0C)
#define IOMUXC_GPR4			(MXC_IOMUXC_BASE + 0x10)
#define IOMUXC_GPR5			(MXC_IOMUXC_BASE + 0x14)
#define IOMUXC_GPR6			(MXC_IOMUXC_BASE + 0x18)
#define IOMUXC_GPR7			(MXC_IOMUXC_BASE + 0x1C)
#define IOMUXC_GPR8			(MXC_IOMUXC_BASE + 0x20)
#define IOMUXC_GPR9			(MXC_IOMUXC_BASE + 0x24)
#define IOMUXC_GPR10			(MXC_IOMUXC_BASE + 0x28)
#define IOMUXC_GPR11			(MXC_IOMUXC_BASE + 0x2C)
#define IOMUXC_GPR12			(MXC_IOMUXC_BASE + 0x30)
#define IOMUXC_GPR13			(MXC_IOMUXC_BASE + 0x34)

/* MMDC */
#define MXC_MMDC_P0_BASE		MX6_IO_ADDRESS(MMDC_P0_BASE_ADDR)
#define MMDC_MDMISC_OFFSET		(MXC_MMDC_P0_BASE + 0x18)
#define MMDC_MDMISC_DDR_TYPE_MASK	(0x3 << 3)
#define MMDC_MDMISC_DDR_TYPE_OFFSET	(3)

/* PLLs */
#define MXC_PLL_BASE			MX6_IO_ADDRESS(ANATOP_BASE_ADDR)
#define PLL1_SYS_BASE_ADDR		(MXC_PLL_BASE + 0x0)
#define PLL2_528_BASE_ADDR		(MXC_PLL_BASE + 0x30)
#define PLL3_480_USB1_BASE_ADDR		(MXC_PLL_BASE + 0x10)
#define PLL4_AUDIO_BASE_ADDR		(MXC_PLL_BASE + 0x70)
#define PLL5_VIDEO_BASE_ADDR		(MXC_PLL_BASE + 0xA0)
#define PLL6_MLB_BASE_ADDR		(MXC_PLL_BASE + 0xD0)
#define PLL7_480_USB2_BASE_ADDR		(MXC_PLL_BASE + 0x20)
#define PLL8_ENET_BASE_ADDR		(MXC_PLL_BASE + 0xE0)
#define PFD_480_BASE_ADDR		(MXC_PLL_BASE + 0xF0)
#define PFD_528_BASE_ADDR		(MXC_PLL_BASE + 0x100)
#define ANADIG_REG_CORE		(MXC_PLL_BASE + 0x140)
#define ANADIG_MISC1_REG		(MXC_PLL_BASE + 0x160)
#define ANATOP_LVDS_CLK1_SRC_SATA	0xB
#define ANATOP_LVDS_CLK1_OBEN_MASK	0x400
#define ANATOP_LVDS_CLK1_IBEN_MASK	0x1000
#define ANA_MISC2_BASE_ADDR		(MXC_PLL_BASE + 0x170)

#define PLL_SETREG_OFFSET		0x4
#define PLL_CLRREG_OFFSET		0x8
#define PLL_TOGGLE_OFFSET		0x0C
#define PLL_NUM_DIV_OFFSET		0x10
#define PLL_DENOM_DIV_OFFSET		0x20
#define PLL_528_SS_OFFSET		0x10
#define PLL_528_NUM_DIV_OFFSET		0x20
#define PLL_528_DENOM_DIV_OFFSET	0x30

/* Common PLL register bit defines. */
#define ANADIG_PLL_LOCK				(1 << 31)
#define ANADIG_PLL_BYPASS			(1 << 16)
#define ANADIG_PLL_BYPASS_CLK_SRC_MASK		(0x3 << 14)
#define ANADIG_PLL_BYPASS_CLK_SRC_OFFSET	(14)
#define ANADIG_PLL_ENABLE			(1 << 13)
#define ANADIG_PLL_POWER_DOWN			(1 << 12)
#define ANADIG_PLL_HOLD_RING_OFF		(1 << 11)

/* PLL1_SYS defines */
#define ANADIG_PLL_SYS_DIV_SELECT_MASK		(0x7F)
#define ANADIG_PLL_SYS_DIV_SELECT_OFFSET	(0)

/* PLL2_528 defines */
#define ANADIG_PLL_528_DIV_SELECT		(1)

/* PLL3_480 defines. */
#define ANADIG_PLL_480_EN_USB_CLKS		(1 << 6)
#define ANADIG_PLL_480_DIV_SELECT_MASK		(0x3)
#define ANADIG_PLL_480_DIV_SELECT_OFFSET	(0)

/* PLL4_AUDIO PLL5_VIDEO defines. */
#define ANADIG_PLL_AV_DIV_SELECT_MASK		(0x7F)
#define ANADIG_PLL_AV_DIV_SELECT_OFFSET		(0)
#define ANADIG_PLL_AV_TEST_DIV_SEL_MASK		(0x180000)
#define ANADIG_PLL_AV_TEST_DIV_SEL_OFFSET	(19)

/* PLL6_MLB defines. */
#define ANADIG_PLL_MLB_LOCK			(1 << 31)
#define ANADIG_PLL_MLB_FLT_RES_CFG_MASK		(0x7 << 26)
#define ANADIG_PLL_MLB_FLT_RES_CFG_OFFSET	(26)
#define ANADIG_PLL_MLB_RX_CLK_DELAY_CFG_MASK	(0x7 << 23)
#define ANADIG_PLL_MLB_RX_CLK_DELAY_CFG_OFFSET	(23)
#define ANADIG_PLL_MLB_VDDD_DELAY_CFG_MASK	(0x7 << 20)
#define ANADIG_PLL_MLB_VDDD_DELAY_CFG_OFFSET	(20)
#define ANADIG_PLL_MLB_VDDA_DELAY_CFG_MASK	(0x7 << 17)
#define ANADIG_PLL_MLB_VDDA_DELAY_CFG_OFFSET	(17)
#define ANADIG_PLL_MLB_BYPASS			(1 << 16)

/* PLL8_ENET defines. */
#define ANADIG_PLL_ENET_LOCK			(1 << 31)
#define ANADIG_PLL_ENET_EN_SATA			(1 << 20)
#define ANADIG_PLL_ENET_EN_PCIE			(1 << 19)
#define ANADIG_PLL_ENET_BYPASS			(1 << 16)
#define ANADIG_PLL_ENET_EN			(1 << 13)
#define ANADIG_PLL_ENET_POWER_DOWN		(1 << 12)
#define ANADIG_PLL_ENET_DIV_SELECT_MASK		(0x3)
#define ANADIG_PLL_ENET_DIV_SELECT_OFFSET	(0)

/* PFD register defines. */
#define ANADIG_PFD_FRAC_MASK			0x3F
#define ANADIG_PFD3_CLKGATE			(1 << 31)
#define ANADIG_PFD3_STABLE			(1 << 30)
#define ANADIG_PFD3_FRAC_OFFSET			24
#define ANADIG_PFD2_CLKGATE			(1 << 23)
#define ANADIG_PFD2_STABLE			(1 << 22)
#define ANADIG_PFD2_FRAC_OFFSET			16
#define ANADIG_PFD1_CLKGATE			(1 << 15)
#define ANADIG_PFD1_STABLE			(1 << 14)
#define ANADIG_PFD1_FRAC_OFFSET			8
#define ANADIG_PFD0_CLKGATE			(1 << 7)
#define ANADIG_PFD0_STABLE			(1 << 6)
#define ANADIG_PFD0_FRAC_OFFSET			0

/* ANATOP  Regulator/LDO defines */
#define ANADIG_REG_RAMP_RATE_MASK			0x03
#define ANADIG_REG_RAMP_RATE_OFFSET			(0x3 << 27)
#define ANADIG_REG_ADJUST_MASK				0xF
#define ANADIG_REG_TARGET_MASK				0x1F
#define ANADIG_REG2_SOC_ADJUST_OFFSET		23
#define ANADIG_REG2_SOC_TARGET_OFFSET		18
#define ANADIG_REG1_PU_ADJUST_OFFSET			14
#define ANADIG_REG1_PU_TARGET_OFFSET			9
#define ANADIG_REG0_CORE_ADJUST_OFFSET		5
#define ANADIG_REG0_CORE_TARGET_OFFSET		0

/* ANA MISC2 register defines */
#define ANADIG_ANA_MISC2_CONTROL3_MASK		0xC0000000
#define ANADIG_ANA_MISC2_CONTROL3_OFFSET	30

#define MXC_CCM_BASE		MX6_IO_ADDRESS(CCM_BASE_ADDR)
/* CCM Register Offsets. */
#define MXC_CCM_CDCR_OFFSET			0x4C
#define MXC_CCM_CACRR_OFFSET			0x10
#define MXC_CCM_CDHIPR_OFFSET			0x48

/* Register addresses of CCM*/
#define MXC_CCM_CCR		(MXC_CCM_BASE + 0x00)
#define MXC_CCM_CCDR		(MXC_CCM_BASE + 0x04)
#define MXC_CCM_CSR		(MXC_CCM_BASE + 0x08)
#define MXC_CCM_CCSR		(MXC_CCM_BASE + 0x0c)
#define MXC_CCM_CACRR		(MXC_CCM_BASE + 0x10)
#define MXC_CCM_CBCDR		(MXC_CCM_BASE + 0x14)
#define MXC_CCM_CBCMR		(MXC_CCM_BASE + 0x18)
#define MXC_CCM_CSCMR1		(MXC_CCM_BASE + 0x1c)
#define MXC_CCM_CSCMR2		(MXC_CCM_BASE + 0x20)
#define MXC_CCM_CSCDR1		(MXC_CCM_BASE + 0x24)
#define MXC_CCM_CS1CDR		(MXC_CCM_BASE + 0x28)
#define MXC_CCM_CS2CDR		(MXC_CCM_BASE + 0x2c)
#define MXC_CCM_CDCDR		(MXC_CCM_BASE + 0x30)
#define MXC_CCM_CHSCCDR		(MXC_CCM_BASE + 0x34)
#define MXC_CCM_CSCDR2		(MXC_CCM_BASE + 0x38)
#define MXC_CCM_CSCDR3		(MXC_CCM_BASE + 0x3c)
#define MXC_CCM_CSCDR4		(MXC_CCM_BASE + 0x40)
#define MXC_CCM_CWDR		(MXC_CCM_BASE + 0x44)
#define MXC_CCM_CDHIPR		(MXC_CCM_BASE + 0x48)
#define MXC_CCM_CDCR		(MXC_CCM_BASE + 0x4c)
#define MXC_CCM_CTOR		(MXC_CCM_BASE + 0x50)
#define MXC_CCM_CLPCR		(MXC_CCM_BASE + 0x54)
#define MXC_CCM_CISR		(MXC_CCM_BASE + 0x58)
#define MXC_CCM_CIMR		(MXC_CCM_BASE + 0x5c)
#define MXC_CCM_CCOSR 		(MXC_CCM_BASE + 0x60)
#define MXC_CCM_CGPR		(MXC_CCM_BASE + 0x64)
#define MXC_CCM_CCGR0		(MXC_CCM_BASE + 0x68)
#define MXC_CCM_CCGR1		(MXC_CCM_BASE + 0x6C)
#define MXC_CCM_CCGR2		(MXC_CCM_BASE + 0x70)
#define MXC_CCM_CCGR3		(MXC_CCM_BASE + 0x74)
#define MXC_CCM_CCGR4		(MXC_CCM_BASE + 0x78)
#define MXC_CCM_CCGR5		(MXC_CCM_BASE + 0x7C)
#define MXC_CCM_CCGR6		(MXC_CCM_BASE + 0x80)
#define MXC_CCM_CCGR7		(MXC_CCM_BASE + 0x84)
#define MXC_CCM_CMEOR		(MXC_CCM_BASE + 0x88)

/* Define the bits in register CCR */
#define MXC_CCM_CCR_RBC_EN			(1 << 27)
#define MXC_CCM_CCR_REG_BYPASS_CNT_MASK		(0x3F << 21)
#define MXC_CCM_CCR_REG_BYPASS_CNT_OFFSET	(21)
#define MXC_CCM_CCR_WB_COUNT_MASK		(0x7 << 16)
#define MXC_CCM_CCR_WB_COUNT_OFFSET		(16)
#define MXC_CCM_CCR_COSC_EN			(1 << 12)
#define MXC_CCM_CCR_OSCNT_MASK			(0xFF)
#define MXC_CCM_CCR_OSCNT_OFFSET		(0)

/* Define the bits in register CCDR */
#define MXC_CCM_CCDR_MMDC_CH1_HS_MASK		(1 << 16)
#define MXC_CCM_CCDR_MMDC_CH0_HS_MASK		(1 << 17)

/* Define the bits in register CSR */
#define MXC_CCM_CSR_COSC_READY			(1 << 5)
#define MXC_CCM_CSR_REF_EN_B			(1 << 0)

/* Define the bits in register CCSR */
#define MXC_CCM_CCSR_PDF_540M_AUTO_DIS		(1 << 15)
#define MXC_CCM_CCSR_PDF_720M_AUTO_DIS		(1 << 14)
#define MXC_CCM_CCSR_PDF_454M_AUTO_DIS		(1 << 13)
#define MXC_CCM_CCSR_PDF_508M_AUTO_DIS		(1 << 12)
#define MXC_CCM_CCSR_PDF_594M_AUTO_DIS		(1 << 11)
#define MXC_CCM_CCSR_PDF_352M_AUTO_DIS		(1 << 10)
#define MXC_CCM_CCSR_PDF_400M_AUTO_DIS		(1 << 9)
#define MXC_CCM_CCSR_STEP_SEL			(1 << 8)
#define MXC_CCM_CCSR_PLL1_SW_CLK_SEL		(1 << 2)
#define MXC_CCM_CCSR_PLL2_SW_CLK_SEL		(1 << 1)
#define MXC_CCM_CCSR_PLL3_SW_CLK_SEL		(1 << 0)

/* Define the bits in register CACRR */
#define MXC_CCM_CACRR_ARM_PODF_OFFSET		(0)
#define MXC_CCM_CACRR_ARM_PODF_MASK		(0x7)

/* Define the bits in register CBCDR */
#define MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK	(0x7 << 27)
#define MXC_CCM_CBCDR_PERIPH_CLK2_PODF_OFFSET	(27)
#define MXC_CCM_CBCDR_PERIPH2_CLK2_SEL		(1 << 26)
#define MXC_CCM_CBCDR_PERIPH_CLK_SEL		(1 << 25)
#define MXC_CCM_CBCDR_MMDC_CH0_PODF_MASK	(0x7 << 19)
#define MXC_CCM_CBCDR_MMDC_CH0_PODF_OFFSET	(19)
#define MXC_CCM_CBCDR_AXI_PODF_MASK		(0x7 << 16)
#define MXC_CCM_CBCDR_AXI_PODF_OFFSET		(16)
#define MXC_CCM_CBCDR_AHB_PODF_MASK		(0x7 << 10)
#define MXC_CCM_CBCDR_AHB_PODF_OFFSET		(10)
#define MXC_CCM_CBCDR_IPG_PODF_MASK		(0x3 << 8)
#define MXC_CCM_CBCDR_IPG_PODF_OFFSET		(8)
#define MXC_CCM_CBCDR_AXI_ALT_SEL_MASK		(1 << 7)
#define MXC_CCM_CBCDR_AXI_ALT_SEL_OFFSET	(7)
#define MXC_CCM_CBCDR_AXI_SEL			(1 << 6)
#define MXC_CCM_CBCDR_MMDC_CH1_PODF_MASK	(0x7 << 3)
#define MXC_CCM_CBCDR_MMDC_CH1_PODF_OFFSET	(3)
#define MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_MASK	(0x7 << 0)
#define MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_OFFSET	(0)

/* Define the bits in register CBCMR */
#define MXC_CCM_CBCMR_GPU3D_SHADER_PODF_MASK		(0x7 << 29)
#define MXC_CCM_CBCMR_GPU3D_SHADER_PODF_OFFSET		(29)
#define MXC_CCM_CBCMR_GPU3D_CORE_PODF_MASK		(0x7 << 26)
#define MXC_CCM_CBCMR_GPU3D_CORE_PODF_OFFSET		(26)
#define MXC_CCM_CBCMR_GPU2D_CORE_PODF_MASK		(0x7 << 23)
#define MXC_CCM_CBCMR_GPU2D_CORE_PODF_OFFSET		(23)
#define MXC_CCM_CBCMR_PRE_PERIPH2_CLK_SEL_MASK		(0x3 << 21)
#define MXC_CCM_CBCMR_PRE_PERIPH2_CLK_SEL_OFFSET	(21)
#define MXC_CCM_CBCMR_PRE_PERIPH2_CLK2_SEL		(1 << 20)
#define MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK		(0x3 << 18)
#define MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_OFFSET		(18)
#define MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK		(0x3 << 16)
#define MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET		(16)
#define MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK		(0x3 << 14)
#define MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET		(14)
#define MXC_CCM_CBCMR_PERIPH_CLK2_SEL_MASK		(0x3 << 12)
#define MXC_CCM_CBCMR_PERIPH_CLK2_SEL_OFFSET		(12)
#define MXC_CCM_CBCMR_VDOAXI_CLK_SEL			(1 << 11)
#define MXC_CCM_CBCMR_PCIE_AXI_CLK_SEL			(1 << 10)
#define MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_MASK		(0x3 << 8)
#define MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_OFFSET	(8)
#define MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_MASK		(0x3 << 4)
#define MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_OFFSET		(4)
#define MXC_CCM_CBCMR_GPU3D_AXI_CLK_SEL			(1 << 1)
#define MXC_CCM_CBCMR_GPU2D_AXI_CLK_SEL			(1 << 0)

/* Define the bits in register CSCMR1 */
#define MXC_CCM_CSCMR1_ACLK_EMI_SLOW_MASK		(0x3 << 29)
#define MXC_CCM_CSCMR1_ACLK_EMI_SLOW_OFFSET		(29)
#define MXC_CCM_CSCMR1_ACLK_EMI_MASK			(0x3 << 27)
#define MXC_CCM_CSCMR1_ACLK_EMI_OFFSET			(27)
#define MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_MASK		(0x7 << 23)
#define MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_OFFSET	(23)
#define MXC_CCM_CSCMR1_ACLK_EMI_PODF_MASK		(0x7 << 20)
#define MXC_CCM_CSCMR1_ACLK_EMI_PODF_OFFSET		(20)
#define MXC_CCM_CSCMR1_USDHC4_CLK_SEL			(1 << 19)
#define MXC_CCM_CSCMR1_USDHC3_CLK_SEL			(1 << 18)
#define MXC_CCM_CSCMR1_USDHC2_CLK_SEL			(1 << 17)
#define MXC_CCM_CSCMR1_USDHC1_CLK_SEL			(1 << 16)
#define MXC_CCM_CSCMR1_SSI3_CLK_SEL_MASK		(0x3 << 14)
#define MXC_CCM_CSCMR1_SSI3_CLK_SEL_OFFSET		(14)
#define MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK		(0x3 << 12)
#define MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET		(12)
#define MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK		(0x3 << 10)
#define MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET		(10)
#define MXC_CCM_CSCMR1_PERCLK_PODF_MASK			(0x3F)
#define MXC_CCM_CSCMR1_PERCLK_PODF_OFFSET		(0)

/* Define the bits in register CSCMR2 */
#define MXC_CCM_CSCMR2_ESAI_CLK_SEL_MASK		(0x3 << 19)
#define MXC_CCM_CSCMR2_ESAI_CLK_SEL_OFFSET		(19)
#define MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV			(1 << 11)
#define MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV			(1 << 10)
#define MXC_CCM_CSCMR2_CAN_CLK_PODF_MASK		(0x3F << 2)
#define MXC_CCM_CSCMR2_CAN_CLK_PODF_OFFSET		(2)

/* Define the bits in register CSCDR1 */
#define MXC_CCM_CSCDR1_VPU_AXI_PODF_MASK		(0x7 << 25)
#define MXC_CCM_CSCDR1_VPU_AXI_PODF_OFFSET		(25)
#define MXC_CCM_CSCDR1_USDHC4_PODF_MASK			(0x7 << 22)
#define MXC_CCM_CSCDR1_USDHC4_PODF_OFFSET		(22)
#define MXC_CCM_CSCDR1_USDHC3_PODF_MASK			(0x7 << 19)
#define MXC_CCM_CSCDR1_USDHC3_PODF_OFFSET		(19)
#define MXC_CCM_CSCDR1_USDHC2_PODF_MASK			(0x7 << 16)
#define MXC_CCM_CSCDR1_USDHC2_PODF_OFFSET		(16)
#define MXC_CCM_CSCDR1_USDHC1_PODF_MASK			(0x7 << 11)
#define MXC_CCM_CSCDR1_USDHC1_PODF_OFFSET		(11)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET		(8)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK		(0x7 << 8)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET		(6)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK		(0x3 << 6)
#define MXC_CCM_CSCDR1_UART_CLK_PODF_MASK		(0x3F)
#define MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET		(0)

/* Define the bits in register CS1CDR */
#define MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK		(0x7 << 25)
#define MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET		(25)
#define MXC_CCM_CS1CDR_SSI3_CLK_PRED_MASK		(0x7 << 22)
#define MXC_CCM_CS1CDR_SSI3_CLK_PRED_OFFSET		(22)
#define MXC_CCM_CS1CDR_SSI3_CLK_PODF_MASK		(0x3F << 16)
#define MXC_CCM_CS1CDR_SSI3_CLK_PODF_OFFSET		(16)
#define MXC_CCM_CS1CDR_ESAI_CLK_PRED_MASK		(0x7 << 9)
#define MXC_CCM_CS1CDR_ESAI_CLK_PRED_OFFSET		(9)
#define MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK		(0x7 << 6)
#define MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET		(6)
#define MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK		(0x3F)
#define MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET		(0)

/* Define the bits in register CS2CDR */
#define MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK		(0x3F << 21)
#define MXC_CCM_CS2CDR_ENFC_CLK_PODF_OFFSET		(21)
#define MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK		(0x7 << 18)
#define MXC_CCM_CS2CDR_ENFC_CLK_PRED_OFFSET		(18)
#define MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK		(0x3 << 16)
#define MXC_CCM_CS2CDR_ENFC_CLK_SEL_OFFSET		(16)
#define MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK		(0x7 << 12)
#define MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET		(12)
#define MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK		(0x7 << 9)
#define MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET		(9)
#define MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK		(0x7 << 6)
#define MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET		(6)
#define MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK		(0x3F)
#define MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET		(0)

/* Define the bits in register CDCDR */
#define MXC_CCM_CDCDR_HSI_TX_PODF_MASK			(0x7 << 29)
#define MXC_CCM_CDCDR_HSI_TX_PODF_OFFSET		(29)
#define MXC_CCM_CDCDR_HSI_TX_CLK_SEL			(1 << 28)
#define MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK		(0x7 << 25)
#define MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET		(25)
#define MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK		(0x7 << 22)
#define MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET		(22)
#define MXC_CCM_CDCDR_SPDIF0_CLK_SEL_MASK		(0x3 << 20)
#define MXC_CCM_CDCDR_SPDIF0_CLK_SEL_OFFSET		(20)
#define MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK		(0x7 << 12)
#define MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET		(12)
#define MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK		(0x7 << 9)
#define MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET		(9)
#define MXC_CCM_CDCDR_SPDIF1_CLK_SEL_MASK		(0x3 << 7)
#define MXC_CCM_CDCDR_SPDIF1_CLK_SEL_OFFSET		(7)

/* Define the bits in register CHSCCDR */
#define MXC_CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_MASK	(0x7 << 15)
#define MXC_CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_OFFSET	(15)
#define MXC_CCM_CHSCCDR_IPU1_DI1_PODF_MASK		(0x7 << 12)
#define MXC_CCM_CHSCCDR_IPU1_DI1_PODF_OFFSET		(12)
#define MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_MASK		(0x7 << 9)
#define MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET		(9)
#define MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK	(0x7 << 6)
#define MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET	(6)
#define MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK		(0x7 << 3)
#define MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET		(3)
#define MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK		(0x7)
#define MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET		(0)

/* Define the bits in register CSCDR2 */
#define MXC_CCM_CSCDR2_ECSPI_CLK_PODF_MASK		(0x3F << 19)
#define MXC_CCM_CSCDR2_ECSPI_CLK_PODF_OFFSET		(19)
#define MXC_CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_MASK	(0x7 << 15)
#define MXC_CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_OFFSET	(15)
#define MXC_CCM_CSCDR2_IPU2_DI1_PODF_MASK		(0x7 << 12)
#define MXC_CCM_CSCDR2_IPU2_DI1_PODF_OFFSET		(12)
#define MXC_CCM_CSCDR2_IPU2_DI1_CLK_SEL_MASK		(0x7 << 9)
#define MXC_CCM_CSCDR2_IPU2_DI1_CLK_SEL_OFFSET		(9)
#define MXC_CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_MASK	(0x7 << 6)
#define MXC_CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_OFFSET	(6)
#define MXC_CCM_CSCDR2_IPU2_DI0_PODF_MASK		(0x7 << 3)
#define MXC_CCM_CSCDR2_IPU2_DI0_PODF_OFFSET		(3)
#define MXC_CCM_CSCDR2_IPU2_DI0_CLK_SEL_MASK		(0x7)
#define MXC_CCM_CSCDR2_IPU2_DI0_CLK_SEL_OFFSET		(0)

/* Define the bits in register CSCDR3 */
#define MXC_CCM_CSCDR3_IPU2_HSP_PODF_MASK		(0x7 << 16)
#define MXC_CCM_CSCDR3_IPU2_HSP_PODF_OFFSET		(16)
#define MXC_CCM_CSCDR3_IPU2_HSP_CLK_SEL_MASK		(0x3 << 14)
#define MXC_CCM_CSCDR3_IPU2_HSP_CLK_SEL_OFFSET		(14)
#define MXC_CCM_CSCDR3_IPU1_HSP_PODF_MASK		(0x7 << 11)
#define MXC_CCM_CSCDR3_IPU1_HSP_PODF_OFFSET		(11)
#define MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_MASK		(0x3 << 9)
#define MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_OFFSET		(9)

/* Define the bits in register CDHIPR */
#define MXC_CCM_CDHIPR_ARM_PODF_BUSY			(1 << 16)
#define MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY		(1 << 5)
#define MXC_CCM_CDHIPR_MMDC_CH0_PODF_BUSY		(1 << 4)
#define MXC_CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY		(1 << 3)
#define MXC_CCM_CDHIPR_MMDC_CH1_PODF_BUSY		(1 << 2)
#define MXC_CCM_CDHIPR_AHB_PODF_BUSY			(1 << 1)
#define MXC_CCM_CDHIPR_AXI_PODF_BUSY			(1)

/* Define the bits in register CLPCR */
#define MXC_CCM_CLPCR_MASK_L2CC_IDLE			(1 << 27)
#define MXC_CCM_CLPCR_MASK_SCU_IDLE			(1 << 26)
#define MXC_CCM_CLPCR_MASK_CORE3_WFI			(1 << 25)
#define MXC_CCM_CLPCR_MASK_CORE2_WFI			(1 << 24)
#define MXC_CCM_CLPCR_MASK_CORE1_WFI			(1 << 23)
#define MXC_CCM_CLPCR_MASK_CORE0_WFI			(1 << 22)
#define MXC_CCM_CLPCR_BYP_MMDC_CH1_LPM_HS		(1 << 21)
#define MXC_CCM_CLPCR_BYP_MMDC_CH0_LPM_HS		(1 << 19)
#define MXC_CCM_CLPCR_WB_CORE_AT_LPM			(1 << 17)
#define MXC_CCM_CLPCR_WB_PER_AT_LPM			(1 << 16)
#define MXC_CCM_CLPCR_COSC_PWRDOWN			(1 << 11)
#define MXC_CCM_CLPCR_STBY_COUNT_MASK			(0x3 << 9)
#define MXC_CCM_CLPCR_STBY_COUNT_OFFSET			(9)
#define MXC_CCM_CLPCR_VSTBY				(1 << 8)
#define MXC_CCM_CLPCR_DIS_REF_OSC			(1 << 7)
#define MXC_CCM_CLPCR_SBYOS				(1 << 6)
#define MXC_CCM_CLPCR_ARM_CLK_DIS_ON_LPM		(1 << 5)
#define MXC_CCM_CLPCR_LPSR_CLK_SEL_MASK			(0x3 << 3)
#define MXC_CCM_CLPCR_LPSR_CLK_SEL_OFFSET		(3)
#define MXC_CCM_CLPCR_BYPASS_PMIC_VFUNC_READY		(1 << 2)
#define MXC_CCM_CLPCR_LPM_MASK				(0x3)
#define MXC_CCM_CLPCR_LPM_OFFSET			(0)

/* Define the bits in register CISR */
#define MXC_CCM_CISR_ARM_PODF_LOADED			(1 << 26)
#define MXC_CCM_CISR_MMDC_CH0_PODF_LOADED		(1 << 23)
#define MXC_CCM_CISR_PERIPH_CLK_SEL_LOADED		(1 << 22)
#define MXC_CCM_CISR_MMDC_CH1_PODF_LOADED		(1 << 21)
#define MXC_CCM_CISR_AHB_PODF_LOADED			(1 << 20)
#define MXC_CCM_CISR_PERIPH2_CLK_SEL_LOADED		(1 << 19)
#define MXC_CCM_CISR_AXI_PODF_LOADED			(1 << 17)
#define MXC_CCM_CISR_COSC_READY				(1 << 6)
#define MXC_CCM_CISR_LRF_PLL				(1)

/* Define the bits in register CIMR */
#define MXC_CCM_CIMR_MASK_ARM_PODF_LOADED		(1 << 26)
#define MXC_CCM_CIMR_MASK_MMDC_CH0_PODF_LOADED		(1 << 23)
#define MXC_CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED		(1 << 22)
#define MXC_CCM_CIMR_MASK_MMDC_CH1_PODF_LOADED		(1 << 21)
#define MXC_CCM_CIMR_MASK_AHB_PODF_LOADED		(1 << 20)
#define MXC_CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED	(1 << 22)
#define MXC_CCM_CIMR_MASK_AXI_PODF_LOADED		(1 << 17)
#define MXC_CCM_CIMR_MASK_COSC_READY			(1 << 6)
#define MXC_CCM_CIMR_MASK_LRF_PLL			(1)

/* Define the bits in register CCOSR */
#define MXC_CCM_CCOSR_CKO2_EN_OFFSET		(24)
#define MXC_CCM_CCOSR_CKO2_DIV_MASK		(0x7 << 21)
#define MXC_CCM_CCOSR_CKO2_DIV_OFFSET		(21)
#define MXC_CCM_CCOSR_CKO2_SEL_OFFSET		(16)
#define MXC_CCM_CCOSR_CKO2_SEL_MASK		(0x1F << 16)
#define MXC_CCM_CCOSR_CKOL_MIRROR_CKO2_MASK	(1 << 8)
#define MXC_CCM_CCOSR_CKOL_EN_OFFSET		7
#define MXC_CCM_CCOSR_CKOL_EN			(0x1 << 7)
#define MXC_CCM_CCOSR_CKOL_DIV_MASK		(0x7 << 4)
#define MXC_CCM_CCOSR_CKOL_DIV_OFFSET		(4)
#define MXC_CCM_CCOSR_CKOL_SEL_MASK		(0xF)
#define MXC_CCM_CCOSR_CKOL_SEL_OFFSET		(0)

/* Define the bits in registers CGPR */
#define MXC_CCM_CGPR_EFUSE_PROG_SUPPLY_GATE	(1 << 4)
#define MXC_CCM_CGPR_MMDC_EXT_CLK_DIS		(1 << 2)
#define MXC_CCM_CGPR_PMIC_DELAY_SCALER		(1)

/* Define the bits in registers CCGRx */
#define MXC_CCM_CCGRx_CG_MASK			0x3
#define MXC_CCM_CCGRx_MOD_OFF			0x0
#define MXC_CCM_CCGRx_MOD_ON			0x3
#define MXC_CCM_CCGRx_MOD_IDLE			0x1

#define MXC_CCM_CCGRx_CG15_MASK			(0x3 << 30)
#define MXC_CCM_CCGRx_CG14_MASK			(0x3 << 28)
#define MXC_CCM_CCGRx_CG13_MASK			(0x3 << 26)
#define MXC_CCM_CCGRx_CG12_MASK			(0x3 << 24)
#define MXC_CCM_CCGRx_CG11_MASK			(0x3 << 22)
#define MXC_CCM_CCGRx_CG10_MASK			(0x3 << 20)
#define MXC_CCM_CCGRx_CG9_MASK			(0x3 << 18)
#define MXC_CCM_CCGRx_CG8_MASK			(0x3 << 16)
#define MXC_CCM_CCGRx_CG5_MASK			(0x3 << 10)
#define MXC_CCM_CCGRx_CG4_MASK			(0x3 << 8)
#define MXC_CCM_CCGRx_CG3_MASK			(0x3 << 6)
#define MXC_CCM_CCGRx_CG2_MASK			(0x3 << 4)
#define MXC_CCM_CCGRx_CG1_MASK			(0x3 << 2)
#define MXC_CCM_CCGRx_CG0_MASK			(0x3 << 0)

#define MXC_CCM_CCGRx_CG15_OFFSET		30
#define MXC_CCM_CCGRx_CG14_OFFSET		28
#define MXC_CCM_CCGRx_CG13_OFFSET		26
#define MXC_CCM_CCGRx_CG12_OFFSET		24
#define MXC_CCM_CCGRx_CG11_OFFSET		22
#define MXC_CCM_CCGRx_CG10_OFFSET		20
#define MXC_CCM_CCGRx_CG9_OFFSET		18
#define MXC_CCM_CCGRx_CG8_OFFSET		16
#define MXC_CCM_CCGRx_CG7_OFFSET		14
#define MXC_CCM_CCGRx_CG6_OFFSET		12
#define MXC_CCM_CCGRx_CG5_OFFSET		10
#define MXC_CCM_CCGRx_CG4_OFFSET		8
#define MXC_CCM_CCGRx_CG3_OFFSET		6
#define MXC_CCM_CCGRx_CG2_OFFSET		4
#define MXC_CCM_CCGRx_CG1_OFFSET		2
#define MXC_CCM_CCGRx_CG0_OFFSET		0

#endif				/* __ARCH_ARM_MACH_MX6_CRM_REGS_H__ */
