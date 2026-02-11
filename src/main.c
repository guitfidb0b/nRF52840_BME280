/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 8 - Define the addresses of relevant registers */
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define ID	 0xD0
#define TEMPMSB	 0xFA
#define PRESSMSB 0xF7

#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x93
#define HUMMEAS  0xF2
#define HUM_CONFIG_VALUE 0x04

/* STEP 6 - Get the node identifier of the sensor */
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
	
	/* Step 10 - Put calibration function code */
	uint8_t values[33];

	int ret = i2c_burst_read_dt(spec, CALIB00, values, 33);

	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
	}

	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];
	sensor_data_ptr->dig_p1 = ((uint16_t)values[7]) << 8 | values[6];
	sensor_data_ptr->dig_p2 = ((uint16_t)values[9]) << 8 | values[8];
	sensor_data_ptr->dig_p3 = ((uint16_t)values[11]) << 8 | values[10];
	sensor_data_ptr->dig_p4 = ((uint16_t)values[13]) << 8 | values[12];
	sensor_data_ptr->dig_p5 = ((uint16_t)values[15]) << 8 | values[14];
	sensor_data_ptr->dig_p6 = ((uint16_t)values[17]) << 8 | values[16];
	sensor_data_ptr->dig_p7 = ((uint16_t)values[19]) << 8 | values[18];
	sensor_data_ptr->dig_p8 = ((uint16_t)values[21]) << 8 | values[20];
	sensor_data_ptr->dig_p9 = ((uint16_t)values[23]) << 8 | values[22];
	sensor_data_ptr->dig_h1 = ((uint8_t)values[24]);
	sensor_data_ptr->dig_h2 = ((uint16_t)values[26]) << 8 | values[25];
	sensor_data_ptr->dig_h3 = ((uint8_t)values[27]);
	sensor_data_ptr->dig_h4 = ((uint16_t)values[29]) << 8 | values[28];
	sensor_data_ptr->dig_h5 = ((uint16_t)values[31]) << 8 | values[30];
	sensor_data_ptr->dig_h6 = ((uint8_t)values[32]);
}

static int32_t bme280_compensate_press(struct bme280_data *data, int32_t adc_press)
{
	int32_t var1, var2, p;
	var1 = t_fine - 128000;
	var2 = var1 * var1 * ((int32_t)data->dig_p6);
	var2 = var2 + ((var1 * (int32_t)data->dig_p5) << 17);
	var2 = var2 + (((int32_t)data->dig_p4) << 35);
	var1 = ((var1 * var1 * (int32_t)data->dig_p3) >> 8) + ((var1 * (int32_t)data->dig_p2) << 12);
	var1 = ((((int32_t)1) << 47) + var1) * ((int32_t)data->dig_p1) >> 33;
	if(var1 == 0)
	{
		return 0;
	}
	p = 1048576 - adc_press;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int32_t)data->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int32_t)data->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int32_t)data->dig_p7) << 4);
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

	return ((var1 + var2) * 5 + 128) >> 8;
}

static int32_t bme280_compensate_hum(struct bme280_data *data, int32_t adc_hum)
{
	int32_t var;
	var = (t_fine - (int32_t)76800);
	var = (((((adc_hum << 14) - (((int32_t)data->dig_h4) << 20) - (((int32_t)data->dig_h5) * var)) + 
	((int32_t)16384)) >> 15) * (((((((var * ((int32_t)data->dig_h6)) >> 10) * (((var * ((int32_t)data->dig_h3)) >> 11) + 
	((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)data->dig_h2) + 
	8192) >> 14));
	var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)data->dig_h1)) >> 4));
	var = (var < 0 ? 0 : var);
	var = (var > 419430400 ? 419430400 : var);
	return var;
}

int main(void)
{

	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

	printk("device addr is %x\n", dev_i2c.addr);

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return -1;
	}

	/* STEP 9 - Verify it is proper device by reading device id  */
	uint8_t id = 0;
	uint8_t regs[] = {ID};

	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);
	//printk("ret: %d\n", ret);
	//int ret = i2c_write_dt(&dev_i2c, regs, 1);
	//if(ret != 0){
	//	printk("Failed to write register %x \n", regs[0]);
	//	return -1;
	//}

	//ret = i2c_read_dt(&dev_i2c, &id, 1);
	printk("Return value: %d\n", ret);
	if (ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
		return -1;
	}

	if (id != CHIP_ID) {
		printk("Invalid chip id! %x \n", id);
		return -1;
	}

	bme_calibrationdata(&dev_i2c, &bmedata);

	/* STEP 11 - Setup the sensor by writing the value 0x93 to the Configuration register */
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

	while (1) {

		/* STEP 12 - Read the temperature from the sensor */
		uint8_t press_val[8] = {0};

		int ret = i2c_burst_read_dt(&dev_i2c, PRESSMSB, press_val, 8);

		if (ret != 0) {
			printk("Failed to read register %x \n", PRESSMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}

		//printk("PressVal1: %d, PressVal2: %d, PressVal3: %d\n", press_val[0], press_val[1], press_val[2]);
		//printk("TempVal1: %d, TempVal2: %d, TempVal3: %d\n", press_val[3], press_val[4], press_val[5]);
		//printk("HumVal1: %d, HumVal2: %d\n", press_val[6], press_val[7]);

		/* STEP 12.1 - Put the data read from registers into actual order (see datasheet) */
		int32_t adc_press = (press_val[0] << 12) | (press_val[1] << 4) | ((press_val[2] >> 4) & 0x0F);
		int32_t adc_temp = (press_val[3] << 12) | (press_val[4] << 4) | ((press_val[5] >> 4) & 0x0F);
		int16_t adc_hum = (press_val[6] << 8) | (press_val[7]);

		//printk("ADCPress: %d\n", adc_press);
		//printk("ADCTemp: %d\n", adc_temp);
		//printk("ADCHum: %d\n", adc_hum);

		/* STEP 12.2 - Compensate temperature */
		int32_t comp_press = bme280_compensate_press(&bmedata, adc_press);
		int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
		int32_t comp_hum = bme280_compensate_hum(&bmedata, adc_hum);

		/* STEP 12.3 - Convert temperature */
		float temperature = (float)comp_temp / 100.0f;
		double fTemp = (double)temperature * 1.8 + 32;
		double pressure = (double)comp_press / 256.0f;
		double humidity = (double)comp_hum / 1024.0f;

		// Print reading to console
		//printk("Temperature in Celsius : %8.2f C\n", (double)temperature);
		printk("Temperature in Fahrenheit : %.2f F\n", fTemp);
		printk("Pressure in Pascals: %.2f Pa\n", pressure);
		printk("Humidity: %.2f percent\n", humidity);

		k_msleep(SLEEP_TIME_MS);
	}
}
