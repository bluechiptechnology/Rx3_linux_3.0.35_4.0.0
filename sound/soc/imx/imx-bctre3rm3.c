/*
 * rx51.c  --  SoC audio for Nokia RX-51
 *
 * Copyright (C) 2008 - 2009 Nokia Corporation
 *
 * Contact: Peter Ujfalusi <peter.ujfalusi@nokia.com>
 *          Eduardo Valentin <eduardo.valentin@nokia.com>
 *          Jarkko Nikula <jhnikula@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>
//#include <plat/mcbsp.h>
#include <mach/audmux.h>
#include "imx-ssi.h"

#include <asm/mach-types.h>

//#include "omap-mcbsp.h"
//#include "omap-pcm.h"



static int bctre3rm3_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	//dev_err(NULL, "bctre3rm3_startup\n");

	snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_CHANNELS, 2, 2);

	return 0;
}

static int rx51_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int channels = params_channels(params);
	int err;


	/* Set codec DAI configuration */
	err = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
	{
		printk("snd_soc_dai_set_fmt codec_dai failed\n");
		return err;
	}


	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	/* Set cpu DAI configuration */
	err = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
	{
		printk("snd_soc_dai_set_fmt cpu_dai failed\n");
		return err;
	}

	/* Set the codec system clock for DAC and ADC */
	return snd_soc_dai_set_sysclk(codec_dai, 0, 6000000,
				      SND_SOC_CLOCK_IN);
}

static struct snd_soc_ops bctre3rm3_ops = {
	.startup = bctre3rm3_startup,
	.hw_params = rx51_hw_params,
};


static int bctre3rm3_aic34_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	//dev_err(NULL, "bctre3rm3_aic34_init\n");

	/* Set up NC codec pins */
	snd_soc_dapm_nc_pin(dapm, "LINE1L");
	snd_soc_dapm_nc_pin(dapm, "LINE1R");
	snd_soc_dapm_nc_pin(dapm, "HPLCOM");
	snd_soc_dapm_nc_pin(dapm, "HPRCOM");
	snd_soc_dapm_nc_pin(dapm, "RLOUT");
	snd_soc_dapm_nc_pin(dapm, "MONO_LOUT");

	snd_soc_dapm_sync(dapm);

	return err;
}

/* Digital audio interface glue - connects codec <--> CPU */
#ifdef CONFIG_BCT_USE_HB_CODEC
static struct snd_soc_dai_link bctre3rm3_dai[] = {
	{
		.name = "TLV320AIC34",
		.stream_name = "AIC34",
		.cpu_dai_name = "imx-ssi.1",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name = "imx-pcm-audio.1",
		.codec_name = "tlv320aic3x-codec.1-0018",
		.init = bctre3rm3_aic34_init,
		.ops = &bctre3rm3_ops,
	},
};
#else
static struct snd_soc_dai_link bctre3rm3_dai[] = {
	{
		.name = "TLV320AIC34",
		.stream_name = "AIC34",
		.cpu_dai_name = "imx-ssi.1",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name = "imx-pcm-audio.1",
		.codec_name = "tlv320aic3x-codec.2-0018",
		.init = bctre3rm3_aic34_init,
		.ops = &bctre3rm3_ops,
	},
};
#endif

/* Audio card */
static struct snd_soc_card bctre3rm3_sound_card = {
	.name = "BCTRE3RM3-TLV320AIC",
	.dai_link = &bctre3rm3_dai,
	.num_links = ARRAY_SIZE(bctre3rm3_dai),
};

static struct platform_device *bctre3rm3_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __init bctre3rm3_soc_init(void)
{
	int err;

	printk("bctre3rm3_soc_init\n");

	imx_audmux_config(2 ,5);

	bctre3rm3_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bctre3rm3_snd_device) 
	{
		dev_err(NULL, "bctre3rm3_soc_init: platform_device_alloc failed\n");
		err = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(bctre3rm3_snd_device, &bctre3rm3_sound_card);

	err = platform_device_add(bctre3rm3_snd_device);
	if (err)
	{
		dev_err(NULL, "bctre3rm3_soc_init: platform_device_add failed\n");
		goto err2;
	}

	return 0;
err2:
	platform_device_put(bctre3rm3_snd_device);
err1:

	return err;
}

static void __exit bctre3rm3_soc_exit(void)
{
	platform_device_unregister(bctre3rm3_snd_device);
}

module_init(bctre3rm3_soc_init);
module_exit(bctre3rm3_soc_exit);

MODULE_AUTHOR("D Robinson Blue Chip Technology");
MODULE_DESCRIPTION("BCTRE3RM3");
MODULE_LICENSE("GPL");
