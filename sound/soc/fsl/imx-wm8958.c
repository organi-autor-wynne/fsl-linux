/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/wm8994/registers.h>
#include "../fsl/fsl_sai.h"
#include "../codecs/wm8994.h"

#define DAI_NAME_SIZE	32

struct imx_wm8958_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *mclk;
	unsigned int clk_frequency;
	bool is_codec_master;
	int sr_stream[2];
};

struct imx_priv {
	int hp_gpio;
	int hp_active_low;
	struct snd_soc_codec *codec;
	struct platform_device *pdev;
	struct snd_kcontrol *headphone_kctl;
	struct snd_card *snd_card;
};

static struct imx_priv card_priv;

static struct snd_soc_jack imx_hp_jack;

static struct snd_soc_jack_pin imx_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio imx_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 250,
	.invert = 1,
};

static int hpjack_status_check(void)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	char *envp[3], *buf;
	int hp_status, ret;

	if (!gpio_is_valid(priv->hp_gpio))
		return 0;

	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdev->dev, "%s kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (hp_status != priv->hp_active_low) {
		snprintf(buf, 32, "STATE=%d", 2);
		snd_soc_dapm_disable_pin(&priv->codec->dapm, "Ext Spk");
		ret = imx_hp_jack_gpio.report;
		snd_kctl_jack_report(priv->snd_card, priv->headphone_kctl, 1);
	} else {
		snprintf(buf, 32, "STATE=%d", 0);
		snd_soc_dapm_enable_pin(&priv->codec->dapm, "Ext Spk");
		ret = 0;
		snd_kctl_jack_report(priv->snd_card, priv->headphone_kctl, 0);
	}

	envp[0] = "NAME=headphone";
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	return ret;
}

static const struct snd_soc_dapm_widget imx_wm8958_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route imx_wm8958_dapm_route[] = {
	{"Headphone Jack", NULL, "HPOUT1L"},
	{"Headphone Jack", NULL, "HPOUT1R"},
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct device *dev = card->dev;
	struct imx_wm8958_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int pll_out;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, data->dai.dai_fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, data->dai.dai_fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	data->clk_frequency = clk_get_rate(data->mclk);

	data->sr_stream[tx] = params_rate(params);

	/*
	 * If the sample rate of second stream is equal to the first stream,
	 * just keep the setting and return.
	 */
	if (data->sr_stream[!tx] &&
			data->sr_stream[tx] == data->sr_stream[!tx])
		return 0;

	if (!data->is_codec_master) {
		ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_FLL_SRC_MCLK1,
				data->clk_frequency, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(dev, "failed to set codec sysclk: %d\n", ret);
			return ret;
		}
	} else {
		/*
		 * Set pll helper function will not set FLL when FLL source,
		 * FLL in and out frequency is not been changed. So for first
		 * stream, we should enable FLL.
		 */
		if (data->sr_stream[!tx] == 0)
			snd_soc_update_bits(codec, WM8994_FLL1_CONTROL_1, 1, 1);

		pll_out = data->sr_stream[tx] * 256;

		ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1,
					  WM8994_FLL_SRC_MCLK1,
					  data->clk_frequency,
					  pll_out);
		if (ret) {
			dev_err(dev, "failed to set codec pll: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
				pll_out, SND_SOC_CLOCK_OUT);
		if (ret) {
			dev_err(dev, "failed to set codec sysclk: %d\n", ret);
			return ret;
		}
	}

	/*
	 * Set GPIO1 pin function to reserve, so that DAC1 and ADC1 using shared
	 * LRCLK from DACLRCK1.
	 */
	snd_soc_update_bits(codec, WM8994_GPIO_1, 0x1f, 0x2);
	return 0;
}

static int imx_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8958_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	/*
	 * Disable FLL1 after all stream finished.
	 */
	if (data->sr_stream[!tx] == 0 && data->sr_stream[tx])
		snd_soc_update_bits(codec, WM8994_FLL1_CONTROL_1, 1, 0);

	data->sr_stream[tx] = 0;

	return 0;
}

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8958_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	if (!IS_ERR(data->mclk)) {
		ret = clk_prepare_enable(data->mclk);
		if (ret) {
			dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8958_data *data = snd_soc_card_get_drvdata(card);

	if (!IS_ERR(data->mclk))
		clk_disable_unprepare(data->mclk);
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
	.hw_free   = imx_hifi_hw_free,
	.startup   = imx_hifi_startup,
	.shutdown  = imx_hifi_shutdown,
};

static int imx_wm8958_gpio_init(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct imx_priv *priv = &card_priv;
	int ret;
	priv->codec = codec;

	if (gpio_is_valid(priv->hp_gpio)) {
		imx_hp_jack_gpio.gpio = priv->hp_gpio;
		imx_hp_jack_gpio.jack_status_check = hpjack_status_check;

		ret = snd_soc_jack_new(codec, "Headphone Jack",
				SND_JACK_HEADPHONE, &imx_hp_jack);
		if (ret)
			return ret;
		ret = snd_soc_jack_add_pins(&imx_hp_jack,
			ARRAY_SIZE(imx_hp_jack_pins), imx_hp_jack_pins);
		if (ret)
			return ret;
		ret = snd_soc_jack_add_gpios(&imx_hp_jack, 1,
						&imx_hp_jack_gpio);
		if (ret)
			return ret;
	}

	return 0;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int hp_status;

	if (!gpio_is_valid(priv->hp_gpio)) {
		strcpy(buf, "no detect gpio connected\n");
		return strlen(buf);
	}

	/* Check if headphone is plugged in */
	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;

	if (hp_status != priv->hp_active_low)
		strcpy(buf, "headphone\n");
	else
		strcpy(buf, "speaker\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static int imx_wm8958_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_wm8958_data *data;
	int ret;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(np, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(np, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	if (of_property_read_bool(np, "codec-master")) {
		data->dai.dai_fmt = SND_SOC_DAIFMT_CBM_CFM;
		data->is_codec_master = true;
	} else
		data->dai.dai_fmt = SND_SOC_DAIFMT_CBS_CFS;

	data->mclk = devm_clk_get(&codec_dev->dev, "mclk1");
	if (IS_ERR(data->mclk)) {
		ret = PTR_ERR(data->mclk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	priv->hp_gpio = of_get_named_gpio_flags(np, "hp-det-gpios", 0,
			(enum of_gpio_flags *)&priv->hp_active_low);

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "wm8994-aif1";
	data->dai.codec_name = "wm8994-codec";
	data->dai.ignore_pmdown_time = 1;
	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
	data->dai.ops = &imx_hifi_ops;
	data->dai.dai_fmt |= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_wm8958_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8958_dapm_widgets);
	data->card.dapm_routes = imx_wm8958_dapm_route;
	data->card.num_dapm_routes = ARRAY_SIZE(imx_wm8958_dapm_route);
	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	priv->snd_card = data->card.snd_card;

	priv->headphone_kctl = snd_kctl_jack_new("Headphone", 0, NULL);
	ret = snd_ctl_add(data->card.snd_card, priv->headphone_kctl);
	if (ret)
		goto fail;

	ret = imx_wm8958_gpio_init(&data->card);

	if (gpio_is_valid(priv->hp_gpio)) {
		ret = driver_create_file(pdev->dev.driver,
						&driver_attr_headphone);
		if (ret) {
			dev_err(&pdev->dev,
					"create hp attr failed (%d)\n", ret);
			goto fail;
		}
	}

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_wm8958_remove(struct platform_device *pdev)
{
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
	return 0;
}

static const struct of_device_id imx_wm8958_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8958", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8958_dt_ids);

static struct platform_driver imx_wm8958_driver = {
	.driver = {
		.name = "imx-wm8958",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8958_dt_ids,
	},
	.probe = imx_wm8958_probe,
	.remove = imx_wm8958_remove,
};
module_platform_driver(imx_wm8958_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8958 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8958");
