/*
 * Author: Li Peng
 * Date: 01-10-2014
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/input.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif

static unsigned char pca9555_key2code[] = {
	KEY_KPASTERISK, KEY_7,          KEY_4,          KEY_1,
	KEY_0,          KEY_8,          KEY_5,          KEY_2,
	KEY_M,          KEY_9,          KEY_6,          KEY_3,
	KEY_RESERVED,   KEY_RESERVED,   KEY_RESERVED,   KEY_RESERVED,
};

static u8 pin_config[2] = {0xff,0x0f};
static u8 led_on[2] = {0xff,0x0f};
static u8 led_off[2] = {0xff,0xff};
static struct i2c_client * temp_client;//if have time just change it
struct delayed_work led_off_work;
unsigned int led_last_time = 1000;
#define LED_LAST_TIME 1000

struct pca9555_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work dwork;
	spinlock_t lock;        /* Protects canceling/rescheduling of dwork */
	unsigned char keycodes[ARRAY_SIZE(pca9555_key2code)];
	u16 key_matrix;
    unsigned int debounce_time;
};

static int pca9555_read_block(struct i2c_client *client,
			     u8 *buffer)
{
	int error;
	u8 command_name[1];

	/*
	 * Can't use SMBus block data read. Check for I2C functionality to speed
	 * things up whenever possible. Otherwise we will be forced to read
	 * sequentially.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))	{
	    command_name[0] = 0;
	    error = i2c_master_send(client, command_name, 1);
    	if (error < 0) {
	    	dev_err(&client->dev,
		    	"!!!!!************couldn't send request. Returned %d\n", error);
		    return error;
	    }
	
    error = i2c_master_recv(client, buffer, 2);
	if (error < 0) {
		dev_err(&client->dev,
			"&&&&&&&&&&&&&&&&&couldn't recv request. Returned %d\n", error);
		return error;
	}

	dev_dbg(&client->dev, "@@@@@@@@@@@@@@@@@@Send %d messages of keyboard by i2c\n", error);
	} else {
		return -1;
	}

	return 0;
}

static int pca9555_write_word(struct i2c_client *client,
			     u8 command,u8 *buffer)
{
	int error;
    u8 send_reg[3];
	/*
	 * Can't use SMBus block data read. Check for I2C functionality to speed
	 * things up whenever possible. Otherwise we will be forced to read
	 * sequentially.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))	{
        send_reg[0] = command;
        send_reg[1] = *(buffer);
        send_reg[2] = *(buffer+1);
    
        error = i2c_master_send(client, send_reg, 3);
    	if (error < 0) {
	    	dev_err(&client->dev,
		    	"$$$$************couldn't send request. Returned %d\n", error);
		return error;
	    }

    dev_dbg(&client->dev, "@@@@@@@@@@@@@@@@@@Send %d messages of keyboard by i2c\n", error);

	} else {
		return -1;
	}

	return 0;
}

static int pca9555_init_key_direction(struct i2c_client *client,u8 *output_config)
{
    int error;
    u8 command = 0x6;
    u8 reg[2];
    reg[0] = *(output_config);
    reg[1] = *(output_config+1);
    
    error = pca9555_write_word(client,command,reg);
	if (error < 0) {
		dev_err(&client->dev,
			"************couldn't send request. Returned %d\n", error);
		return error;
	}
    
    return 0;
}

static int pca9555_init_key_polarity(struct i2c_client *client,u8 *output_config)
{
    int error;
    u8 command = 0x4;
    u8 reg[2];
    reg[0] = *(output_config);
    reg[1] = *(output_config+1);
    
    error = pca9555_write_word(client,command,reg);
	if (error < 0) {
		dev_err(&client->dev,
			"************couldn't send request. Returned %d\n", error);
		return error;
	}
    
    return 0;
}

static int pca9555_led_on(struct i2c_client *client,u8 *output_config)
{
    int error;
    u8 command = 0x2;
    u8 reg[2];
    reg[0] = *(output_config);
    reg[1] = *(output_config+1);
    
    error = pca9555_write_word(client,command,reg);
	if (error < 0) {
		dev_err(&client->dev,
			"************couldn't send request. Returned %d\n", error);
		return error;
	}
    
    return 0;
}

static int pca9555_led_off(struct i2c_client* client, u8 *output_config)
{
    int error;
    u8 command = 0x2;
    u8 reg[2];
    reg[0] = *(output_config);
    reg[1] = *(output_config+1);
    
    error = pca9555_write_word(client,command,reg);
	if (error < 0) {
		dev_err(&client->dev,
			"************couldn't send request. Returned %d\n", error);
		return error;
	}
   
    return 0;
}

static void pca9555_led_worker(struct work_struct *work)
{
    pca9555_led_off(temp_client,led_off);
}

static int pca9555_get_key_matrix(struct pca9555_data *pca9555)
{
	struct i2c_client *client = pca9555->client;
	struct input_dev *input = pca9555->input;
	u8 regs[2];
	u8 reg_change_times[16];
	u16 old_matrix, new_matrix;
	int ret, i, mask;

	dev_dbg(&client->dev, "requesting keys...\n");
	/*
	 * Read all registers from General Status Register
	 * to GPIOs register
	 */
	memset(reg_change_times, 0, 16);
	
	ret = pca9555_read_block(client, regs);
	if (ret) {
        dev_err(&client->dev,
					"could not perform chip read.\n");
			return ret;
	}

	old_matrix = pca9555->key_matrix;
    new_matrix = (regs[1] << 8) | regs[0];

	mask = 0x0001;
	for (i = 0; i < 12; ++i, mask <<= 1) 
    {
		int keyval = new_matrix & mask;
		if ((old_matrix & mask) != keyval) 
        {
			input_report_key(input, pca9555->keycodes[i], keyval);
			pca9555->key_matrix = new_matrix;
			dev_dbg(&client->dev, "key %d %s\n",
						i, keyval ? "pressed" : "released");
		}
       else 
        {
			reg_change_times[i] = 0;
	    }
	}
	input_sync(input);

    pca9555_led_on(client,led_on);
	__cancel_delayed_work(&led_off_work);
	schedule_delayed_work(&led_off_work,  msecs_to_jiffies(led_last_time));
	
    return 0;
}

static irqreturn_t pca9555_irq(int irq, void *_pca9555)
{
	struct pca9555_data *pca9555 = _pca9555;
	unsigned long flags;
    
    spin_lock_irqsave(&pca9555->lock, flags);
	__cancel_delayed_work(&pca9555->dwork);
	schedule_delayed_work(&pca9555->dwork,  msecs_to_jiffies(pca9555->debounce_time));
	spin_unlock_irqrestore(&pca9555->lock, flags);
	
    return IRQ_HANDLED;
}

static void pca9555_worker(struct work_struct *work)
{
	struct pca9555_data *pca9555 =
		container_of(work, struct pca9555_data, dwork.work);

	dev_dbg(&pca9555->client->dev, "worker\n");
	pca9555_get_key_matrix(pca9555);

}

static int pca9555_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct pca9555_data *pca9555;
	struct input_dev *input;
	int i;
	int error;
    int gpio,ret;
    u8 *test = pin_config;
    int wakeup=0;

    struct device *dev = &client->dev;
    
    /* Check functionality */
	error = i2c_check_functionality(client->adapter,
            I2C_FUNC_I2C);
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	/* Chip is valid and active. Allocate structure */
	pca9555 = kzalloc(sizeof(struct pca9555_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!pca9555 || !input) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	pca9555->client = client;
	pca9555->input = input;
	// Support button is on at begin
	pca9555->key_matrix = 0x0000;
	INIT_DELAYED_WORK(&pca9555->dwork, pca9555_worker);
	INIT_DELAYED_WORK(&led_off_work, pca9555_led_worker);
    spin_lock_init(&pca9555->lock);

	input->name = "PCA9555 Touch Sense Keyboard";
	input->id.bustype = BUS_I2C;

	input->keycode = pca9555->keycodes;
	input->keycodesize = sizeof(pca9555->keycodes[0]);
	input->keycodemax = ARRAY_SIZE(pca9555_key2code);
	for (i = 0; i < BITS_TO_LONGS(KEY_CNT); i++) {
		input->key[i] = 0x00000000;
	}

	__set_bit(EV_KEY, input->evbit);
	__clear_bit(EV_REP, input->evbit);
	for (i = 0; i < ARRAY_SIZE(pca9555_key2code); i++) {
		pca9555->keycodes[i] = pca9555_key2code[i];
		__set_bit(pca9555_key2code[i], input->keybit);
	}
	__clear_bit(KEY_RESERVED, input->keybit);

    ret = of_property_read_u32(dev->of_node, "debounce_time",
					&(pca9555->debounce_time));

    ret = of_property_read_u32(dev->of_node, "led_last_time",
					&(led_last_time));
    
    gpio = of_get_named_gpio(dev->of_node,"keypad_int", 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "no gpio pin available\n");
		return -ENODEV;
	}
	ret = gpio_request(gpio, dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to request gpio pin: %d\n", ret);
		goto err_free_mem;
	}
	ret = gpio_direction_input(gpio);
	if (ret) {
		dev_err(dev, "Failed to set gpio to input: %d\n", ret);
		goto err_free_mem;
	}

    client->irq = gpio_to_irq(gpio);
    
    wakeup =1;
    /* Calibrate device */
	if (client->irq) {
		error = request_irq(client->irq, pca9555_irq,
				    IRQF_TRIGGER_FALLING, "pca9555", pca9555);
		if (error) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", client->irq);
			goto err_free_mem;
		}
        else
        {
            if(wakeup){
                enable_irq_wake(client->irq);
            }
        }
    }

	error = input_register_device(pca9555->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device\n");
		goto err_free_irq;
	}

	i2c_set_clientdata(client, pca9555);
    
    temp_client = client; 
    pca9555_init_key_direction(client,test);
    pca9555_init_key_polarity(client,test);

    device_init_wakeup(dev, wakeup);

    return 0;

err_free_irq:
	if (client->irq)
		free_irq(client->irq, pca9555);
err_free_mem:
	input_free_device(input);
	kfree(pca9555);
	return error;
}

static int pca9555_remove(struct i2c_client *client)
{
	struct pca9555_data *pca9555 = i2c_get_clientdata(client);

	/* Release IRQ so no queue will be scheduled */
	if (client->irq)
		free_irq(client->irq, pca9555);

	cancel_delayed_work_sync(&pca9555->dwork);

	input_unregister_device(pca9555->input);
	kfree(pca9555);

	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_device_id pca9555_idtable[] = {
	{ "pca9555", 0, },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pca9555_idtable);

static const struct of_device_id pca953x_dt_ids[] = {
	{ .compatible = "nxp,pca9555_keypad", },
	{ },
};
MODULE_DEVICE_TABLE(of, pca953x_dt_ids);

static struct i2c_driver pca9555_driver = {
	.driver = {
		.name	= "pca9555",
		.owner  = THIS_MODULE,
		.of_match_table = pca953x_dt_ids,
	},

	.id_table	= pca9555_idtable,
	.probe		= pca9555_probe,
	.remove		= pca9555_remove,
};

static int __init pca9555_init(void)
{
    return i2c_add_driver(&pca9555_driver);
}
module_init(pca9555_init);

static void __exit pca9555_cleanup(void)
{
	i2c_del_driver(&pca9555_driver);
}
module_exit(pca9555_cleanup);

MODULE_AUTHOR("Home System <Peng.li3@honeywell.com>");
MODULE_DESCRIPTION("Driver for keyborad");
MODULE_LICENSE("GPL");
