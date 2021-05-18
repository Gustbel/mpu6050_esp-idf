#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "mpu6050.h"

int interrupt_threshold = 30;  // From 0 to 255

float x, y, z, accel;

short value;
i2c_cmd_handle_t cmd;

bool check_mpu6050()
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, TICK_DELAY);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    	return true;
    else
    	return false;
}

void init_mpu6050()
{
	slave_write_byte(SMPLRT_DIV, 0x07);
	slave_write_byte(CONFIG, 0);
	slave_write_byte(GYRO_CONFIG, 24);
} 

void init_int_mpu6050() {

  	slave_write_byte(PWR_MGMT_1, 0x00);
  	slave_write_byte(SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  	slave_write_byte(I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  	slave_write_byte(ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  	slave_write_byte(MOT_THR, interrupt_threshold); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  	slave_write_byte(MOT_DUR, 40); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
  	slave_write_byte(MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
  	slave_write_byte(INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
	vTaskDelay(15/portTICK_PERIOD_MS);

}


void print_mpu6050()
{
    if (check_mpu6050())
	printf("MPU6050 : OK");
    else
	printf("MPU6050 : Not detected (Accelerometer, Gyroscope sensor)");
}

float get_mpu6050()
{
    x = ((slave_read_byte(ACCEL_XOUT_H) << 8) | slave_read_byte(ACCEL_XOUT_H + 1) ) / 1638.4;
    y = ((slave_read_byte(ACCEL_YOUT_H) << 8) | slave_read_byte(ACCEL_YOUT_H + 1) ) / 1638.4;
    z = ((slave_read_byte(ACCEL_ZOUT_H) << 8) | slave_read_byte(ACCEL_ZOUT_H + 1) ) / 1638.4;

    accel = sqrt( pow(x,2) + pow(y,2) + pow(z,2) );

    return accel;
}

float get_mpu6050_axis(int a)
{
    switch (a) 
    {	
    	case 0:	/* Aceler X */
           	return ( ((slave_read_byte(ACCEL_XOUT_H) << 8) | slave_read_byte(ACCEL_XOUT_H + 1) ) / 1638.4 );
        case 1:	/* Aceler Y */
    	    return ( ((slave_read_byte(ACCEL_YOUT_H) << 8) | slave_read_byte(ACCEL_YOUT_H + 1) ) / 1638.4 );
        case 2:
		    return ( ((slave_read_byte(ACCEL_ZOUT_H) << 8) | slave_read_byte(ACCEL_ZOUT_H + 1) ) / 1638.4 );
        case 3:
    	    return ( ((slave_read_byte(GYRO_XOUT_H) << 8) | slave_read_byte(GYRO_XOUT_H + 1) ) / 131.0 );
        case 4:
    	    return ( ((slave_read_byte(GYRO_YOUT_H) << 8) | slave_read_byte(GYRO_YOUT_H + 1) ) / 131.0 );
        case 5:
    	    return ( ((slave_read_byte(GYRO_ZOUT_H) << 8) | slave_read_byte(GYRO_ZOUT_H + 1) ) / 131.0 );
    }
    return 0;
}


void reset_int() {

	printf("INTERRUPTION: Register 0x3A: %d (0x%x)\n", slave_read_byte(0x3A), slave_read_byte(0x3A));
	vTaskDelay(15/portTICK_PERIOD_MS);

}

bool slave_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, TICK_DELAY);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return false;
    }
    return true;
}

int8_t slave_read_byte(uint8_t addr) 
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, TICK_DELAY);
    i2c_cmd_link_delete(cmd);

    uint8_t buf;
    int8_t d;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | 1, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &buf, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, TICK_DELAY);
    i2c_cmd_link_delete(cmd);

    
    d=buf;
    return d;
}
