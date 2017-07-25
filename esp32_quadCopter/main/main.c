// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "gattsCom.h"
#include "imuso3.h"
#include "imu.h"
#include "motor.h"
#include "esp32delay.h"
#include "control.h"
#include "commApp.h"
#include "altitude.h"
#include "battery.h"
#include "led.h"
#include "fbm320.h"
#include "I2Cdev.h"

#include "sdkconfig.h"

SemaphoreHandle_t imuSemaphore;

static void imuTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;

    IMU_Init();

    while (1) {
    	ReadIMUSensorHandle();
    	if(imu.caliFlag)
    	{
    		imu.caliFlag = IMUCalibrate();
    	}
    	else
    	{
    		IMUSO3Update();
    		xSemaphoreGive(imuSemaphore);
    	}
        vTaskDelay( 3*( task_idx + 1 ) / portTICK_RATE_MS);
    }
}



static void copterTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;
    Motor_init();
    while (1) {
    	if(xSemaphoreTake(imuSemaphore, ( TickType_t )5) == pdTRUE ){
        	CtrlAttiRate();
        	CtrlMotor();
    	}
    }
}



static void baroTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;

    fbm320_init();

    while (1) {
    	Fbm320Process();
    	if(!imu.caliFlag){
    		AltitudeCombineUpdate(fbm320.Altitude);
    	}
    }

}


static void comTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;

    while (1) {
    	if(xSemaphoreTake( bleSemaphore, ( TickType_t )15) == pdTRUE ){
       		RCDataReceive();
//        	ESP_LOGI(GATTS_TAG, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//       			bleData[0], bleData[1], bleData[2], bleData[3], bleData[4], bleData[5], bleData[6], bleData[7]);
    	}
    	RCDataLost();
    	RCDataProcess();
    	CtrlAltitude();
    	CtrlAttiAng();
    }
}



static void safeTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;

    LedInit();
    while (1) {
    	BatteryCheck(motorState.enable);
    	ledState.event = E_READY;

    	if(motorState.enable)
    		ledState.event = E_MOTOR_ENABLE;

    	if(!imu.caliPass)
    		ledState.event = E_CALI_FAIL;

    	if(rcState.lostFlag)
    		ledState.event = E_LOST_RC;

    	if(imu.caliFlag)
    		ledState.event = E_CALI;

    	if(copterState.landOff && !motorState.enable)
    		ledState.event = E_AUTO_LANDED;

    	if(battery.alarm)
    		ledState.event = E_BAT_LOW;

    	if(battery.charge)			//battery charge check
    		ledState.event = E_BatChg;

    	LedFSM();
//   	ESP_LOGI(GATTS_TAG,"The baro: %f, %f, %f, %f, %f, %f\n",fbm320.Altitude, -nav.z, -nav.vz, -nav.az, imu.accb[2],motorState.Thro);
		ESP_LOGI(GATTS_TAG,"%f\t%f\t%f\n", imu.roll, imu.pitch, imu.yaw);
    	vTaskDelay( 100*( task_idx + 1 ) / portTICK_RATE_MS);
    }
}


static void debugTask(void *pvParam)
{
    uint32_t task_idx = (uint32_t) pvParam;
    uint8_t buf[11];
    int16_t buf16;

    while (1) {
    	buf16 = imu.roll*100;
    	buf[0] = buf16&0xFF;
    	buf[1] = buf16>>8;

    	buf16 = imu.pitch*100;
    	buf[2] = buf16&0xFF;
    	buf[3] = buf16>>8;

    	buf16 = imu.yaw*100;
    	buf[4] = buf16&0xFF;
    	buf[5] = buf16>>8;

    	buf16 = -nav.z*100;
    	buf[6] = buf16&0xFF;
    	buf[7] = buf16>>8;

    	buf16 = battery.val*100;
    	buf[8] = buf16&0xFF;
    	buf[9] = buf16>>8;

        gattsSendData(buf, 11);
//    	ESP_LOGI(GATTS_TAG,"%d\t%d\t%d\t%d\t%d\n", adcData[0], adcData[1], adcData[2],adcData[3],adcData[4]);
        vTaskDelay( 1000*( task_idx + 1 ) / portTICK_RATE_MS);
    }
}


//all default value
void ParamSetDefault(void)
{
	pitch_angle_PID.P = 3.5;
	pitch_angle_PID.I = 0;//1.0;		//0
	pitch_angle_PID.D = 0;

	pitch_angle_PID.iLimit = 500;	//or 1000

	pitch_rate_PID.P = 0.7;		//0.7
	pitch_rate_PID.I = 0.5; 		//0.5
	pitch_rate_PID.D = 0.03;		//0.03
	pitch_rate_PID.iLimit = 300;	//300

////////////////////////////////////////////
	roll_angle_PID.P = 3.5;
	roll_angle_PID.I = 0;//1.0;
	roll_angle_PID.D = 0;
	roll_angle_PID.iLimit = 500;	//or 1000

	roll_rate_PID.P = 0.7;			//0.7
	roll_rate_PID.I = 0.5;; 		//0.5
	roll_rate_PID.D = 0.03;			//0.03
	roll_rate_PID.iLimit = 300;		//300
///////////////////////////////////////////
	yaw_angle_PID.P = 1;
	yaw_angle_PID.I = 0.2;
	yaw_angle_PID.D = 0;

	yaw_rate_PID.P = 20;
	yaw_rate_PID.I = 0;
	yaw_rate_PID.D = 0;

	alt_PID.P = 1.0;
	alt_PID.I = 0;
	alt_PID.D = 0;

	alt_vel_PID.P = 0.1f;
	alt_vel_PID.I =0.02f;
	alt_vel_PID.D = 0;
}

/*
void GPIO_init()
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1<<5);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

}*/


void app_main()
{
    esp_err_t ret;

    ESP_ERROR_CHECK(nvs_flash_init());

//    GPIO_init();
    I2C_init();
    BAT_init();
    gatts_app();
    vTaskDelay(100 / portTICK_RATE_MS);

    ParamSetDefault();
    copterState.headMode = 1;
    imu.caliFlag = 1;
    bleSemaphore = xSemaphoreCreateBinary();
    imuSemaphore = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(imuTask, "imuTask", 1024*2, NULL, ESP_TASK_PRIO_MIN+4, NULL, 1);
    xTaskCreatePinnedToCore(baroTask, "baroTask", 1024*3, NULL, ESP_TASK_PRIO_MIN+3, NULL, 1);
    xTaskCreatePinnedToCore(comTask, "comTask", 1024*2, NULL, ESP_TASK_PRIO_MIN+2, NULL, 1);
    xTaskCreatePinnedToCore(safeTask, "safeTask", 1024*2, NULL, ESP_TASK_PRIO_MIN+1, NULL, 1);
    xTaskCreatePinnedToCore(copterTask, "copterTask", 1024*2, NULL, ESP_TASK_PRIO_MIN+5, NULL, 1);

    return;
}
