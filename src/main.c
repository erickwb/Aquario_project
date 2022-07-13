/*

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <machine/ieeefp.h>
#include "driver/uart.h"
#include "string.h"

#define R0 10000 //thermistor resistance at 25 degrees Celsius 10k
#define th_Coeff 3470 //thermistor coefficient
#define Rseries 10000 // 10K series resistor
#define BUF_SIZE (1024)
#define OFF 0
#define ON 1
//define portas
#define tempSensor 36
#define releAquecedo 25
#define releColler 15
#define releBomba 16
#define boiaSensor 17

void vGpioConf();
//handles
xQueueHandle tempQueue;
xQueueHandle printTempQueue;
xTaskHandle TaskHandle = NULL;
xTaskHandle TaskHandle2 = NULL;
xTaskHandle TaskHandle3 = NULL; //taskA
xTaskHandle TaskHandle4 = NULL; //taskB

xSemaphoreHandle semaphore;

char flag_boia;


static void IRAM_ATTR InterruptFunction(void* args){
    if(flag_boia == 'f'){
        gpio_set_level(releBomba,1);
        flag_boia = 'a';
    }else if(flag_boia == 'a'){
       gpio_set_level(releBomba,0);
       flag_boia = 'f';
    }
        
}


void tempMeasurement(void *pvParameters){
    
    float measuere = 0;

    double rez = 0;
    double kel = 0;
    double cel = 0;

    uint32_t val = 0;
    
    adc1_config_width(ADC_WIDTH_12Bit); // 0 .. 4095
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db); //ADC1 Ch1 = IO36
    
    while(1){
        for(int i = 0; i < 10; i++){
          val += adc1_get_raw(ADC1_CHANNEL_0);
          vTaskDelay(2/portTICK_PERIOD_MS);
        }

        val = val/10;
        rez = Rseries/((4095./val) - 1);
        kel = 1./(1./298.15 + 1./th_Coeff * log(rez/R0));
        cel = kel - 273.15;
        
        measuere = cel;          
        xQueueSend(tempQueue, &measuere, portMAX_DELAY); 
        xQueueSend(printTempQueue, &measuere, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));

        val = 0;
    }
}


void controlAtuadores (void *pvParameters){
    //aquecedor cooler 
    float measuere;

    while (1) {
        xQueueReceive(tempQueue, &measuere, portMAX_DELAY);
        if(measuere > 28.00){
            //temp acima de 28 
            gpio_set_level(releAquecedo,0); //aquecedor ON
            gpio_set_level(releColler,1); // coller OFF
        }else if (measuere < 23 ){
            //temp abaixo de 23 
            gpio_set_level(releAquecedo,1);//aquecedor OFF
            gpio_set_level(releColler,0); // coller ON
        }
        else{
            //temp ideal 
            gpio_set_level(releAquecedo,1);//aquecedor OFF
            gpio_set_level(releColler,1);// coller OFF

        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void printTaskA(void *pvParameters){
    float measuere; 

    while (1)
    {
        if(xSemaphoreTake(semaphore,pdMS_TO_TICKS(200)==true)){
            xQueueReceive(printTempQueue, &measuere, portMAX_DELAY);
            printf("tmp celsius: %f \n", measuere);
            vTaskDelay(pdMS_TO_TICKS(500));
        }else{
            printf("boia\n");
            
            xSemaphoreGive( semaphore );
        }

        
    }
    
 
}

void floaterMeasurement(void *pvParameters){
    
    while (1)
    {
        gpio_install_isr_service(0);
        gpio_isr_handler_add(boiaSensor,InterruptFunction,(void*) boiaSensor);  
    }
    
}



void app_main() {
    
    vGpioConf();
    flag_boia = 'f';
    xTaskCreate(&tempMeasurement, "temperature", configMINIMAL_STACK_SIZE+1024, NULL, 1, &TaskHandle);
    xTaskCreate(&controlAtuadores,"Atuadores",configMINIMAL_STACK_SIZE+1024,NULL,1,&TaskHandle2);
    xTaskCreate(&printTaskA,"pintTemp", configMINIMAL_STACK_SIZE+1024,NULL,1,&TaskHandle3);
   // xTaskCreate(&floaterMeasurement,"pinttest", configMINIMAL_STACK_SIZE+1024,NULL,1,&TaskHandle4);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(boiaSensor,InterruptFunction,(void*) boiaSensor);  
    tempQueue = xQueueCreate(1,sizeof(float *)); 
    printTempQueue = xQueueCreate(1,sizeof(float *));
    semaphore = xSemaphoreCreateBinary();

}


void vGpioConf(){
    //rele1 aquecedor
    gpio_config_t GPIOconfig;
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << releAquecedo);
    gpio_config(&GPIOconfig);

    //rele2 coller 
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << releColler);
    gpio_config(&GPIOconfig);

    //rele3 LED
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << releBomba);
    gpio_set_level(releBomba,1);
    gpio_config(&GPIOconfig);

    //sensor temp
    GPIOconfig.mode = GPIO_MODE_INPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << tempSensor);
    gpio_config(&GPIOconfig);

    //sensor boia
    GPIOconfig.mode = GPIO_MODE_INPUT;
    GPIOconfig.intr_type = GPIO_INTR_ANYEDGE;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (15 << boiaSensor);
    gpio_config(&GPIOconfig);


}