/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define SLEEP_TIME_MS 1000
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define CALIB26  0xE1
#define ID	 0xD0
#define TEMPMSB	 0xFA
#define PRESSMSB 0xF7
#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x27
#define HUMMEAS  0xF2
#define HUM_CONFIG_VALUE 0x01
#define SOFTRST  0xE0
#define RESET_CONFIG_VALUE 0xB6

#define I2C_NODE DT_NODELABEL(bme280)

/* Data structure to store BME280 data */
struct bme280_data {
	/* Compensation parameters */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;
} bmedata;

int32_t t_fine;

/* Read sensor calibration data and stores these into sensor data */
void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	uint8_t values_l[24];
	int ret = i2c_burst_read_dt(spec, CALIB00, values_l, 24);
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}

	uint8_t values_h[9];
	ret = i2c_burst_read_dt(spec, CALIB26, values_h, 9);
	if (ret != 0){
		printk("Failed to read register %x \n", CALIB26);
		return;
	}

	printk("Reading comp values: \n");
	//Problem with values is here
	sensor_data_ptr->dig_t1 = (uint16_t)((uint16_t)values_l[1] << 8 | values_l[0]);
	sensor_data_ptr->dig_t2 = (int16_t)((int16_t)values_l[3] << 8 | values_l[2]);
	sensor_data_ptr->dig_t3 = (int16_t)((int16_t)values_l[5] << 8 | values_l[4]);
	sensor_data_ptr->dig_p1 = (uint16_t)((uint16_t)values_l[7] << 8 | values_l[6]);
	sensor_data_ptr->dig_p2 = (int16_t)((int16_t)values_l[9] << 8 | values_l[8]);
	sensor_data_ptr->dig_p3 = (int16_t)((int16_t)values_l[11] << 8 | values_l[10]);
	sensor_data_ptr->dig_p4 = (int16_t)((int16_t)values_l[13] << 8 | values_l[12]);
	sensor_data_ptr->dig_p5 = (int16_t)((int16_t)values_l[15] << 8 | values_l[14]);
	sensor_data_ptr->dig_p6 = (int16_t)((int16_t)values_l[17] << 8 | values_l[16]);
	sensor_data_ptr->dig_p7 = (int16_t)((int16_t)values_l[19] << 8 | values_l[18]);
	sensor_data_ptr->dig_p8 = (int16_t)((int16_t)values_l[21] << 8 | values_l[20]);
	sensor_data_ptr->dig_p9 = (int16_t)((int16_t)values_l[23] << 8 | values_l[22]);
	sensor_data_ptr->dig_h1 = ((uint8_t)values_h[0]);
	sensor_data_ptr->dig_h2 = (int16_t)((int16_t)values_h[2] << 8 | values_h[1]);
	sensor_data_ptr->dig_h3 = ((uint8_t)values_h[3]);
	sensor_data_ptr->dig_h4 = (int16_t)((int16_t)values_h[5] << 8 | values_h[4]);
	sensor_data_ptr->dig_h5 = (int16_t)((int16_t)values_h[7] << 8 | values_h[6]);
	sensor_data_ptr->dig_h6 = ((int8_t)values_h[8]);
	for(int i = 0; i<24; i++){
		printk("value %d: %d\n", i, values_l[i]);
	}
	for(int i = 0; i<9; i++){
		printk("value %d: %d\n", i, values_h[i]);
	}
}

static int32_t bme280_compensate_press(struct bme280_data *data, int32_t adc_press)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)data->dig_p6);
	var2 = var2 + ((var1*((int32_t)data->dig_p5))<<1);
	var2 = (var2>>2) + (((int32_t)data->dig_p4)<<16);
	var1 = (((data->dig_p3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)data->dig_p2) * 
		var1)>>1)) >>18;
	var1 = ((((32768+var1)) * ((int32_t)data->dig_p1))>>15);
	if(var1 == 0){
		return 0; //Avoid dividing by 0
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_press)-(var2>>12)))*3125;
	if(p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
	}
	else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)data->dig_p9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)data->dig_p8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + data->dig_p7) >> 4));
	return p;
}

/* Compensate current temperature using previously stored sensor calibration data */
static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;
	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;
	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;
	//set global t_fine for use in press/hum
	t_fine = var1 + var2;
	return (t_fine * 5 + 128) >> 8;
}

static int32_t bme280_compensate_hum(struct bme280_data *data, int32_t adc_hum)
{
	int32_t var;
	var = (t_fine - ((int32_t)76800));
	var = (((((adc_hum << 14) - (((int32_t)data->dig_h4) << 20) - (((int32_t)data->dig_h5) * 
		var)) + ((int32_t)16384)) >> 15) * (((((((var * 
		((int32_t)data->dig_h6)) >> 10) * (((var * ((int32_t)data->dig_h3)) >> 11) + 
		((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)data->dig_h2) + 
		8192) >> 14));
	var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)data->dig_h1)) >> 4));
	var = (var < 0 ? 0 : var);
	var = (var > 419430400 ? 419430400 : var);
	printk("*** In function var: %d\n", var >> 12);
	return (uint32_t)(var >> 12);
}


int main(void)
{
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	//printk("device addr is %x\n", dev_i2c.addr);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return -1;
	}

	uint8_t id = 0;
	uint8_t regs[] = {ID};
	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);	
	//printk("Return value: %d\n", ret);
	if (ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
		return -1;
	}
	if (id != CHIP_ID) {
		printk("Invalid chip id! %x \n", id);
		return -1;
	}

	//try soft reset prior to getting calibration data
	uint8_t soft_reset[] = {SOFTRST, RESET_CONFIG_VALUE};
	ret = i2c_write_dt(&dev_i2c, soft_reset, 2);
	if (ret != 0) {
		printk("Failed to write register %x \n", soft_reset[0]);
		return -1;
	}

	uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};
	ret = i2c_write_dt(&dev_i2c, sensor_config, 2);
	if (ret != 0) {
		printk("Failed to write register %x \n", sensor_config[0]);
		return -1;
	}

	uint8_t hum_config[] = {HUMMEAS, HUM_CONFIG_VALUE};
	ret = i2c_write_dt(&dev_i2c, hum_config, 2);
	if (ret != 0) {
		printk("Failed to write register %x \n", hum_config[0]);
		return -1;
	}

	bme_calibrationdata(&dev_i2c, &bmedata);

	while (1) {
		uint8_t press_val[8] = {0};
		int ret = i2c_burst_read_dt(&dev_i2c, PRESSMSB, press_val, 8);
		if (ret != 0) {
			printk("Failed to read register %x \n", PRESSMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}
		int32_t adc_temp = (press_val[3] << 12) | (press_val[4] << 4) | ((press_val[5] >> 4) & 0x0F);
		int32_t adc_press = (press_val[0] << 12) | (press_val[1] << 4) | ((press_val[2] >> 4) & 0x0F);		
		int32_t adc_hum = (press_val[6] << 8) | (press_val[7]);

		//printk("*** hum val: %d  %d\n", press_val[6], press_val[7]);
		//printk("*** adc Hum: %d\n", adc_hum);
		for(int i =0; i<8; i++){
			printk("*** val %d: %d\n", i, press_val[i]);
		}

		//printk("ADCHum: %d\n", adc_hum);
		int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
		int32_t comp_press = bme280_compensate_press(&bmedata, adc_press);
		int32_t comp_hum = bme280_compensate_hum(&bmedata, adc_hum);

		float temperature = (float)comp_temp / 100.0f;
		double fTemp = (double)temperature * 1.8 + 32;
		double pressure = (double)comp_press * 0.0001450377;
		double humidity = (double)comp_hum / 1024.0;
		// Print reading to console
		printk("Temperature in Fahrenheit : %.2f F\n", fTemp);
		printk("Pressure in PSI: %.2f PSI\n", pressure);
		printk("Humidity: %.2f percent\n", humidity);
		printk("Humidity (raw): %d\n\n", comp_hum);
		k_msleep(SLEEP_TIME_MS);
	}
}
