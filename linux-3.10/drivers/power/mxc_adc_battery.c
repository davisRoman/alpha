/*
 *	iPAQ h1930/h1940/rx1950 battery controller driver
 *	Copyright (c) Vasily Khoruzhick
 *	Based on h1940_battery.c by Arnaud Patard
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/power/mxc_adc_battery.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>

#define BAT_POLL_INTERVAL       (2*HZ)

#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/module.h>

#include <linux/mxc_adc.h>
#include <linux/slab.h>
#define MXC_TX_BUF_SIZE 1
#define MXC_RX_BUF_SIZE 2

#define ADC_SAMPLES     5
struct board_power_ctrl_gpio board_power_ctrl;

struct mxc_adc_bat {
    struct platform_device       *plf_dev;
    struct power_supply     psy;
    struct mxc_adc_bat_platform_data    *pdata;
    unsigned long           volt_value;
    int                     low_bat;
    int                     over_temp;
    struct delayed_work     low_bat_work;
    struct delayed_work     over_temp_work;
    int                     gpio_low_bat;
    int                     enable_over_temp;
    int                     gpio_over_temp;
    int                     low_bat_active_low;
    int                     over_temp_active_low;
    int                     volt_mult;
    int timer_debounce;
    unsigned int adc_id;
    unsigned int adc_ch;
};

struct mxc_adc_bat *main_bat;

static enum power_supply_property mxc_adc_main_bat_props[] = {
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CAPACITY,
};

static int mxc_adc_bat_get_voltage(struct mxc_adc_bat *bat)
{
    int ret;
    int i = 0;
    unsigned long adc_value = 0,result = 0;
    if(bat->plf_dev == NULL){
        dev_err(&bat->plf_dev->dev, "no plf dev ?\n");
        return -ESHUTDOWN;
    }
read_adc:
    ret = tsscve_adc_read(bat->adc_id ,bat->adc_ch ,&adc_value);

    if(ret < 0){
        dev_err(&bat->plf_dev->dev, "adc read fail %d\n",ret);
        return ret;
    }
   
    result += adc_value;
    if(++i < ADC_SAMPLES){
        msleep(1);
        goto read_adc;
    }
    result /= ADC_SAMPLES;

    result = result /243 * 1243;
    if(bat->volt_mult){
        bat->volt_value = result + bat->volt_mult ; // add 0.08v compensation
    }else
    {
        bat->volt_value = result ;
    }
    
    return 0;
}

static int mxc_adc_bat_get_property(struct power_supply *psy,
            enum power_supply_property psp,
            union power_supply_propval *val)
{
    if (!main_bat) {
        dev_err(psy->dev, "no battery infos ?!\n");
        return -EINVAL;
    }

    switch (psp) {
        case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
          val->intval = main_bat->low_bat;
          return 0;
        case POWER_SUPPLY_PROP_TEMP:
          val->intval = main_bat->over_temp;
          return 0;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
          mxc_adc_bat_get_voltage(main_bat);
          val->intval = main_bat->volt_value;
          return 0;
        case POWER_SUPPLY_PROP_PRESENT:
          val->intval = 1;
          return 0;
        case POWER_SUPPLY_PROP_CAPACITY:
          val->intval = 100;
          return 0;
        default:
          return -EINVAL;
    }
}

static void mxc_adc_bat_low_bat_work(struct work_struct *work)
{
    main_bat->low_bat =(gpio_get_value(main_bat->gpio_low_bat)?1:0);
    main_bat->low_bat ^= main_bat->low_bat_active_low;
    power_supply_changed(&main_bat->psy);
}

static void mxc_adc_bat_over_temp_work(struct work_struct *work)
{
    main_bat->over_temp = (gpio_get_value(main_bat->gpio_over_temp)?1:0);
    main_bat->over_temp ^= main_bat->over_temp_active_low;
    power_supply_changed(&main_bat->psy);
}

static irqreturn_t mxc_adc_bat_low_bat_irq(int irq, void *devid)
{
    __cancel_delayed_work(&main_bat->low_bat_work);
    schedule_delayed_work(&main_bat->low_bat_work,
                msecs_to_jiffies(main_bat->timer_debounce));

    return IRQ_HANDLED;
}

static irqreturn_t mxc_adc_bat_over_temp_irq(int irq, void *devid)
{
    __cancel_delayed_work(&main_bat->over_temp_work);
    schedule_delayed_work(&main_bat->over_temp_work,
                msecs_to_jiffies(main_bat->timer_debounce));

    return IRQ_HANDLED;
}

static int mxc_adc_bat_probe(struct platform_device *plf_dev)
{
    int ret;
    int irq;
    struct device *dev = &plf_dev->dev;

    struct mxc_adc_bat_platform_data *pdata = kzalloc(sizeof(struct  mxc_adc_bat_platform_data), GFP_KERNEL);
    main_bat = kzalloc(sizeof(struct mxc_adc_bat), GFP_KERNEL);
    if (!main_bat)
      return -ENOMEM;

    pdata->gpio_low_bat = of_get_named_gpio(dev->of_node,"GPIO_LOW_BAT", 0);
	if (!gpio_is_valid(pdata->gpio_low_bat)) {
		dev_err(dev, "no pdata->gpio_low_bat pin available\n");
		return -ENODEV;
	}

    ret = of_property_read_u32(dev->of_node, "enable_over_temp",
					&(pdata->enable_over_temp));
   
    if(pdata->enable_over_temp == 0)
    {
		pdata->gpio_over_temp = 0;
        pdata->over_temp_active_low = 0;
    }
    
    ret = of_property_read_u32(dev->of_node, "low_bat_active_low",
					&(pdata->low_bat_active_low));
    if(ret < 0)
    {
        pr_err("channel is out of range \n");
        return -ENODEV;
    }

    ret = of_property_read_u32(dev->of_node, "volt_mult",
					&(pdata->volt_mult));
    if(ret < 0)
    {
        pr_warn("volt_mult is not set! \n");
    }
    ret = of_property_read_u32(dev->of_node, "wakeup",
					&(pdata->wakeup));
    
    ret = of_property_read_u32(dev->of_node, "debounce_interval",
					&(pdata->debounce_interval));

    ret = of_property_read_u32(dev->of_node, "adc_id",
					&(pdata->adc_id));
    if(ret < 0)
    {
        pr_err("need adc_id ! \n");
        return -ENODEV;
    }

    ret = of_property_read_u32(dev->of_node, "adc_ch",
					&(pdata->adc_ch));
    if(ret < 0)
    {
        pr_err("adc_ch is not set \n");
        return -ENODEV;
    }

//  initialize the power ctrl gpio for reboot and poweroff
    board_power_ctrl.ac_loss = of_get_named_gpio(dev->of_node,"ac_loss", 0);
	if (!gpio_is_valid(board_power_ctrl.ac_loss)) {
		dev_err(dev, "no ac_loss pin available\n");
	}
    
    board_power_ctrl.batt_test_n = of_get_named_gpio(dev->of_node,"batt_test_n", 0);
	if (!gpio_is_valid(board_power_ctrl.batt_test_n)) {
		dev_err(dev, "no batt_test_n available\n");
	}
    
    board_power_ctrl.bat_ot_ind = of_get_named_gpio(dev->of_node,"bat_ot_ind", 0);
	if (!gpio_is_valid(board_power_ctrl.bat_ot_ind)) {
		dev_err(dev, "no bat_ot_ind pin available\n");
	}
    
    board_power_ctrl.charge_disable = of_get_named_gpio(dev->of_node,"charge_disable", 0);
	if (!gpio_is_valid(board_power_ctrl.charge_disable)) {
		dev_err(dev, "no charge_disable pin available\n");
	}
    
    board_power_ctrl.usb_4g = of_get_named_gpio(dev->of_node,"usb_4g", 0);
	if (!gpio_is_valid(board_power_ctrl.usb_4g)) {
		dev_err(dev, "no usb_4g pin available\n");
	}
  //  if(gpio_request(board_power_ctrl.usb_4g, "usb_4g") == 0){
  //      gpio_direction_output(board_power_ctrl.usb_4g, 0);
  //  }
   // gpio_free(board_power_ctrl.usb_4g);
    
    board_power_ctrl.npgate_zw = of_get_named_gpio(dev->of_node,"npgate_zw", 0);
	if (!gpio_is_valid(board_power_ctrl.npgate_zw)) {
		dev_err(dev, "no npgate_zw pin available\n");
	}

// end of gpio set
    main_bat->plf_dev = plf_dev;
    main_bat->pdata = pdata;
    main_bat->volt_value = 0;
    main_bat->psy.name = "BAT";
    main_bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
    main_bat->psy.properties = mxc_adc_main_bat_props;
    main_bat->psy.num_properties = ARRAY_SIZE(mxc_adc_main_bat_props);
    main_bat->psy.get_property = mxc_adc_bat_get_property;
    main_bat->gpio_low_bat = pdata->gpio_low_bat;
    main_bat->enable_over_temp = pdata->enable_over_temp;
    main_bat->gpio_over_temp = pdata->gpio_over_temp;
    main_bat->low_bat_active_low = pdata->low_bat_active_low;
    main_bat->over_temp_active_low = pdata->over_temp_active_low;
    main_bat->volt_mult = pdata->volt_mult;
    main_bat->adc_id = pdata->adc_id; 
    main_bat->adc_ch = pdata->adc_ch; 


    ret = gpio_request(pdata->gpio_low_bat, "mxc_low_bat");
    if (ret) {
        dev_err(&plf_dev->dev, "Failed to request gpio pin: %d\n", ret);
        goto err_free;
    }
    ret = gpio_direction_input(pdata->gpio_low_bat);
    if (ret) {
        dev_err(&plf_dev->dev, "Failed to set gpio to input: %d\n", ret);
        gpio_free(pdata->gpio_low_bat);
    }

    if (main_bat->enable_over_temp == 1) {
        ret = gpio_request(pdata->gpio_over_temp, "mxc_over_temp");
        if (ret) {
            dev_err(&plf_dev->dev, "Failed to request gpio pin: %d\n", ret);
            goto err_free;
        }
        ret = gpio_direction_input(pdata->gpio_over_temp);
        if (ret) {
            dev_err(&plf_dev->dev, "Failed to set gpio to input: %d\n", ret);
            gpio_free(pdata->gpio_over_temp);
        }
    }

    ret = power_supply_register(&plf_dev->dev, &main_bat->psy);
    if (ret){
        goto err_free;
    }

    main_bat->low_bat = (gpio_get_value(pdata->gpio_low_bat)?1:0);
    main_bat->low_bat ^= pdata->low_bat_active_low;

    if (main_bat->enable_over_temp == 1) {
        main_bat->over_temp = (gpio_get_value(pdata->gpio_over_temp)?1:0);
        main_bat->over_temp ^= pdata->over_temp_active_low;
    }

    irq = gpio_to_irq(pdata->gpio_low_bat);
    if (irq > 0) {
        ret = request_any_context_irq(irq, mxc_adc_bat_low_bat_irq,
                    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                    "mxc_adc_bat low bat", main_bat);
        if (ret)
          dev_warn(&plf_dev->dev, "Failed to request irq: %d\n", ret);
        else{
            if(pdata->wakeup){
                enable_irq_wake(irq);
            }
        }
    }

    if (main_bat->enable_over_temp == 1) {
        irq = gpio_to_irq(pdata->gpio_over_temp);
        if (irq > 0) {
            ret = request_any_context_irq(irq, mxc_adc_bat_over_temp_irq,
                                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                                          "mxc_adc_bat over temp", main_bat);
            if (ret)
                dev_warn(&plf_dev->dev, "Failed to request irq: %d\n", ret);
            else{
                if(pdata->wakeup){
                    enable_irq_wake(irq);
                }
            }
        }
    }

    main_bat->timer_debounce = TIMER_DEBOUNCE;
    if(pdata->debounce_interval){
        main_bat->timer_debounce = pdata->debounce_interval;
    }
    INIT_DELAYED_WORK(&main_bat->low_bat_work, mxc_adc_bat_low_bat_work);
    if (main_bat->enable_over_temp == 1) {
        INIT_DELAYED_WORK(&main_bat->over_temp_work, mxc_adc_bat_over_temp_work);
    }
    device_init_wakeup(&plf_dev->dev, 1);
    dev_info(&plf_dev->dev, "successfully loaded\n");
    return 0;

err_free:
    kfree(main_bat);
    return ret;
}

static int mxc_adc_bat_remove(struct platform_device *plf_dev)
{
    device_init_wakeup(&plf_dev->dev, 0);
    power_supply_unregister(&main_bat->psy);
    gpio_free(main_bat->pdata->gpio_low_bat);
    if (main_bat->enable_over_temp == 1) {
        gpio_free(main_bat->pdata->gpio_over_temp);
        cancel_delayed_work(&main_bat->over_temp_work);
    }
    cancel_delayed_work(&main_bat->low_bat_work);
    return 0;
}

static int mxc_adc_bat_resume(struct device *dev)
{
    main_bat->low_bat = (gpio_get_value(main_bat->gpio_low_bat)?1:0);
    main_bat->low_bat ^= main_bat->low_bat_active_low;
    power_supply_changed(&main_bat->psy);

    return 0;
}

static SIMPLE_DEV_PM_OPS(mxc_adc_bat_pm_ops, NULL, mxc_adc_bat_resume);

static const struct of_device_id of_mxc_id[] = {
    { .compatible = "ti,mxc-adc-battery"},
    { },
};
MODULE_DEVICE_TABLE(of, of_mxc_id);

static struct platform_driver mxc_adc_bat_driver = {
    .driver = {
        .name	= "mxc-adc-battery",
        .owner	= THIS_MODULE,
        .pm = &mxc_adc_bat_pm_ops,
        .of_match_table = of_mxc_id,
    },
    .probe		= mxc_adc_bat_probe,
    .remove		= mxc_adc_bat_remove,
};

static int __init mxc_adc_bat_init(void)
{
    return platform_driver_register(&mxc_adc_bat_driver);
}
module_init(mxc_adc_bat_init);

static void __exit mxc_adc_bat_exit(void)
{
    platform_driver_unregister(&mxc_adc_bat_driver);
}
module_exit(mxc_adc_bat_exit);

MODULE_DESCRIPTION("mxc adc battery driver");
MODULE_LICENSE("GPL");
