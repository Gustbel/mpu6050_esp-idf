#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_tls.h"

#include "i2c.h"
#include "src/mpu6050.h"

void app_main(void) {

    printf("\n	-- MPU6050 lib test --\n");	

    // Init I2C
    init_i2c();
    
    // Init sensor

    // Check sensor
    if (check_mpu6050()){
        printf("\nInitialized!!!\n");
    }
    else{
        printf("\nNO Initialized\n");
    }

    // Read data
    while(1){
        printf("Accelerometer:\tX: %f", get_mpu6050_old(0));
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("\tY: %f", get_mpu6050_old(1)); 
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("\tZ: %f\n", get_mpu6050_old(2));
        vTaskDelay(15/portTICK_PERIOD_MS);
        
        printf("Gyroscope:\tX: %f", get_mpu6050_old(3));
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("\tY: %f", get_mpu6050_old(4)); 
        vTaskDelay(15/portTICK_PERIOD_MS);
        printf("\tZ: %f\n\n\n", get_mpu6050_old(5));
        vTaskDelay(15/portTICK_PERIOD_MS);

        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}
