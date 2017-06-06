/*
 * mxc adc battery driver
 *
 * Copyright (C) 2013 honeywell co,.ltd
 * Li Song <Song.li@honeywell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __MXC_ADC_BATTERY_H_
#define __MXC_ADC_BATTERY_H_

#define TIMER_DEBOUNCE 10 /*default debounce time*/
struct mxc_adc_bat_platform_data {
    int gpio_low_bat;    /*low bat interrupt gpio*/
    int low_bat_active_low;    /*low bat active low flag*/
    int enable_over_temp; /*indication for whether over temp function enabled;0-disabled,1-enabled*/
    int gpio_over_temp;  /*over temperature interrupt gpio*/
    int over_temp_active_low;    /*over temperature active low flag*/
    int wakeup;
    int debounce_interval;
    unsigned int volt_mult;   /*voltage multiplier*/
    
    unsigned int adc_id;
    unsigned int adc_ch;
};


struct board_power_ctrl_gpio{
   int ac_loss;
   int batt_test_n;
   int bat_ot_ind;
   int charge_disable; 
   int usb_4g;
   int npgate_zw;
};



#endif /* __mxc_ADC_BATTERY_H_ */
