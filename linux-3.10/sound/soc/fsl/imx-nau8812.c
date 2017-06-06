/*
 * imx-nau8812.c
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/delay.h>

#include "imx-audmux.h"
#include "../codecs/nau8812.h"

struct imx_priv {
    int sysclk;         /*mclk from the outside*/
    int npgate_gpio;
    struct platform_device *pdev;
};
unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static struct imx_priv card_priv;
struct clk *nau8812_mclk;

static int imx_audmux_config(int slave, int master)
{
    unsigned int ptcr, pdcr;
    slave = slave - 1;
    master = master - 1;

    ptcr = IMX_AUDMUX_V2_PTCR_SYN |
        IMX_AUDMUX_V2_PTCR_TFSDIR |
        IMX_AUDMUX_V2_PTCR_TFSEL(master) |
        IMX_AUDMUX_V2_PTCR_TCLKDIR |
        IMX_AUDMUX_V2_PTCR_TCSEL(master);
    pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);
    imx_audmux_v2_configure_port(slave, ptcr, pdcr);

    ptcr = IMX_AUDMUX_V2_PTCR_SYN;
    pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);
    imx_audmux_v2_configure_port(master, ptcr, pdcr);

    return 0;
}

static int get_bclk_div(struct snd_pcm_hw_params *params)
{
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S8:
    case SNDRV_PCM_FORMAT_U8:
        return 0x10;
    case SNDRV_PCM_FORMAT_U16:
    case SNDRV_PCM_FORMAT_S16_LE:
    case SNDRV_PCM_FORMAT_S16_BE:
        return 0x0C;
    case SNDRV_PCM_FORMAT_S20_3LE:
    case SNDRV_PCM_FORMAT_S20_3BE:
    case SNDRV_PCM_FORMAT_S24_3LE:
    case SNDRV_PCM_FORMAT_S24_3BE:
    case SNDRV_PCM_FORMAT_S24_BE:
    case SNDRV_PCM_FORMAT_S24_LE:
    case SNDRV_PCM_FORMAT_U24_BE:
    case SNDRV_PCM_FORMAT_U24_LE:
    case SNDRV_PCM_FORMAT_U24_3BE:
    case SNDRV_PCM_FORMAT_U24_3LE:
        return 0x08;//192/48
    case SNDRV_PCM_FORMAT_S32:
    case SNDRV_PCM_FORMAT_U32:
        return 0x08;
    default:
        return 2;
    }
}

static int imx_nau8812_hw_params(struct snd_pcm_substream *substream,
                              struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct imx_priv *priv = &card_priv;
    //struct clk *clko;
    unsigned int channels = params_channels(params);
    unsigned int sample_rate = 44100;
    int ret = 0;
    u32 dai_format;
    unsigned int pll_out;

    /* if (codec->dapm.bias_level != SND_SOC_BIAS_OFF) */
    /*     return 0; */

    dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
        SND_SOC_DAIFMT_CBM_CFM;

    /* set codec DAI configuration */
    ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
    if (ret < 0)
        return ret;

    /* set i.MX active slot mask */
    snd_soc_dai_set_tdm_slot(cpu_dai,
                             channels == 1 ? 0xfffffffe : 0xfffffffc,
                             channels == 1 ? 0xfffffffe : 0xfffffffc,
                             2,
                             32);

    dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
        SND_SOC_DAIFMT_CBM_CFM;
    /* set cpu DAI configuration */
    ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
    if (ret < 0)
        return ret;

    sample_rate = params_rate(params);
    sample_format = params_format(params);

    if (sample_format == SNDRV_PCM_FORMAT_S24_LE)
        pll_out = sample_rate * 192;
    else
        pll_out = sample_rate * 256;

    ret = snd_soc_dai_set_sysclk(codec_dai,
                                 NAU8812_PLL,
                                 priv->sysclk,
                                 SND_SOC_CLOCK_IN);
    if (ret < 0)
        pr_err("Failed to set sysclk: %d\n", ret);
#if 0
    ret = snd_soc_dai_set_clkdiv(codec_dai,
                                 nau8812_256FSRATE,
                                 pll_out);

    if (ret < 0)
        pr_err("Failed to set OPCLKRATE: %d\n", ret);
#endif
    ret = snd_soc_dai_set_clkdiv(codec_dai,
                                 NAU8812_BCLKDIV,
                                 get_bclk_div(params));
    if (ret < 0)
        pr_err("Failed to set BCLKDIV: %d\n", ret);
    return 0;
}

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_nau8812_dapm_widgets[] = {
//    SND_SOC_DAPM_HP("Headphone Jack", NULL),
//    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_MIC("AMIC", NULL),
//    SND_SOC_DAPM_MIC("DMIC", NULL),
};

static struct snd_soc_ops imx_nau8812_ops = {
    .hw_params = imx_nau8812_hw_params,
};

static struct snd_soc_dai_link imx_nau8812_dai[] = {
    {
        .name = "HiFi",
        .stream_name = "HiFi",
        .codec_dai_name    = "nau8812",
        .ops        = &imx_nau8812_ops,
        .dai_fmt = SND_SOC_DAIFMT_I2S |
            SND_SOC_DAIFMT_NB_NF |
            SND_SOC_DAIFMT_CBM_CFM,
    },
};

static struct snd_soc_card snd_soc_card_imx = {
    .name        = "imx-audio-nau8812",
    .dai_link    = imx_nau8812_dai,
    .num_links    = ARRAY_SIZE(imx_nau8812_dai),
    .dapm_widgets = imx_nau8812_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(imx_nau8812_dapm_widgets),
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int imx_nau8812_probe(struct platform_device *pdev)
{
    struct snd_soc_card *card = &snd_soc_card_imx;
    struct device_node *np = pdev->dev.of_node;
    struct device_node *ssi_np, *codec_np;
    struct platform_device *ssi_pdev;
    struct i2c_client *codec_dev;
    struct imx_priv *priv = &card_priv;
    struct clk *codec_clk;
    int int_port, ext_port;
    int ret;

    ret = of_property_read_u32(np, "mux-int-port", &int_port);
    if (ret) {
        dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
        return ret;
    }
    ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
    if (ret) {
        dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
        return ret;
    }

    imx_audmux_config(int_port, ext_port);

    ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
    codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
    if (!ssi_np || !codec_np) {
        dev_err(&pdev->dev, "phandle missing or invalid\n");
        ret = -EINVAL;
        goto fail;
    }

    ssi_pdev = of_find_device_by_node(ssi_np);
    if (!ssi_pdev) {
        dev_err(&pdev->dev, "failed to find SSI platform device\n");
        ret = -EINVAL;
        goto fail;
    }

    codec_dev = of_find_i2c_device_by_node(codec_np);
    if (!codec_dev) {
        dev_err(&pdev->dev, "failed to find codec platform device\n");
        ret = -EINVAL;
        goto fail;
    }
#if 0
    priv->npgate_gpio = of_get_named_gpio(codec_np, "npgate-gpio", 0);
    if(priv->npgate_gpio < 0){
        dev_err(&pdev->dev, "Failed to get npgate-gpio\n");
        ret = -EINVAL;
        goto fail;
    }

    ret = gpio_request(priv->npgate_gpio, "npgate_gpio");
    if(ret){
        dev_err(&pdev->dev, "Failed to request npgate_gpio\n");
        ret = -EINVAL;
        goto fail;
    }
    gpio_direction_output(priv->npgate_gpio, 0);
    udelay(50);
    gpio_free(priv->npgate_gpio);
#endif
    codec_clk = clk_get(&codec_dev->dev, NULL);
    if (IS_ERR(codec_clk)) {
        dev_err(&pdev->dev, "failed to get codec clock\n");
        ret = -EINVAL;
        goto fail;
    } else {
        priv->sysclk = clk_get_rate(codec_clk);
        clk_prepare_enable(codec_clk);
    }

    card->dev = &pdev->dev;
    card->dai_link->codec_of_node = codec_np;
    card->dai_link->cpu_dai_name = dev_name(&ssi_pdev->dev);
    card->dai_link->platform_of_node = ssi_np;
    /*
     *ret = snd_soc_of_parse_card_name(card, "model");
     *if (ret) {
     *    dev_err(&pdev->dev, "failed to get model\n");
     *    ret = -EINVAL;
     *    goto fail;
     *}
     *ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
     *if (ret) {
     *    dev_err(&pdev->dev, "failed to get audio-routing\n");
     *    ret = -EINVAL;
     *    goto fail;
     *}
     */
    platform_set_drvdata(pdev, card);

    ret = snd_soc_register_card(card);
    if (ret)
      dev_err(&pdev->dev, "Failed to register card: %d\n", ret);

fail:
    if (ssi_np)
      of_node_put(ssi_np);
    if (codec_np)
      of_node_put(codec_np);

    return ret;
}

static int imx_nau8812_remove(struct platform_device *pdev)
{
    struct snd_soc_card *card = &snd_soc_card_imx;

    snd_soc_unregister_card(card);

    return 0;
}

static const struct of_device_id imx_nau8812_dt_ids[] = {
    { .compatible = "fsl,imx-audio-nau8812", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_nau8812_dt_ids);

static struct platform_driver imx_nau8812_driver = {
    .driver = {
        .name = "imx-nau8812",
        .owner = THIS_MODULE,
        .of_match_table = imx_nau8812_dt_ids,
    },
    .probe = imx_nau8812_probe,
    .remove = imx_nau8812_remove,
};

module_platform_driver(imx_nau8812_driver);

/* Module information */
MODULE_AUTHOR("Song Li <Song.Li@honeywell.org>");
MODULE_DESCRIPTION("ALSA SoC imx nau8812");
MODULE_LICENSE("GPL v2");
