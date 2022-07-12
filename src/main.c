#include <stdio.h>
#include "string.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <math.h>
#include <machine/ieeefp.h>




//ideias
//semaforo do print da temperatura e do nivel da boia
//



/* DEFINES (parâmetros) do Sensor de Temperatura */
#define R0 10000 //thermistor resistance at 25 degrees Celsius 10k
#define th_Coeff 3470 //thermistor coefficient
#define Rseries 10000 // 10K series resistor
#define BUF_SIZE (1024)//thermistor connected to ground
#define OFF 0
#define ON 1

/* DEFINES das PORTAS GPIO ESP32 */
#define releAquecedor 1
#define releColler 15
#define releBomba 16
#define boiaSensor 17
#define button 18
#define tempSensor 36

void vGpioConf();

/* Handles do FreeRTOS */
xQueueHandle tempQueue;
xQueueHandle printQueue;
xTaskHandle TaskHandle = NULL;
xTaskHandle TaskHandle2 = NULL;
SemaphoreHandle_t xSemaphore_Serial = NULL; //semaphore da boia
SemaphoreHandle_t xSemaphore_ExpIO = NULL;

/* Variável global de interrupção */
char flag_boia;


static void IRAM_ATTR InterruptFunction(void* args){
    if(flag_boia == 'f'){
        gpio_set_level(releBomba,1);
        printf("Rele On");
        flag_boia = 'a';
    }else if(flag_boia == 'a'){
       gpio_set_level(releBomba,0);
       printf("Rele Off");
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
        xQueueSend(printQueue,&measuere,portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));

        val = 0;
    }
}

void print(){
    float measuere;
    while(1){
       // xQueueReceive(printQueue, &measuere, portMAX_DELAY);
        xSemaphoreTake(xSemaphore_Serial,portMAX_DELAY);
        print("temperatura atual em C: %f", measuere);
        vTaskDelay(pdMS_TO_TICKS(20000));
        xSemaphoreGive( xSemaphore_Serial );
    }
}

void controlAtuadores (void *pvParameters){
    //aquecedor cooler 
    float measuere;

    while (1) {
        xQueueReceive(tempQueue, &measuere, portMAX_DELAY);
        if(measuere > 25.00){
            //temp acima de 25 
            gpio_set_level(releAquecedor,ON); //aquecedor ON
            gpio_set_level(releColler,OFF); // coller OFF
        }else if (measuere < 23 ){
            //temp abaixo de 23 
            gpio_set_level(releAquecedor,OFF);//aquecedor OFF
            gpio_set_level(releColler,ON); // coller ON
        }
        else{
            //temp ideal 
            gpio_set_level(releAquecedor,OFF);//aquecedor OFF
            gpio_set_level(releColler,OFF);// coller OFF

        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void controlaBomba(void *pvParameters){
    

}

void app_main() {
    
    vGpioConf();
    xTaskCreate(&tempMeasurement, "temperature", configMINIMAL_STACK_SIZE+1024, NULL, 1, &TaskHandle);
    xTaskCreate(&controlAtuadores,"Atuadores",configMINIMAL_STACK_SIZE+1024,NULL,1,&TaskHandle2);
    tempQueue = xQueueCreate(1,sizeof(float *)); 
    printQueue = xQueueCreate(1,sizeof(float *));
    flag_boia = 'f';
    gpio_install_isr_service(0);
    gpio_isr_handler_add(button,InterruptFunction,(void*) button);
   // gpio_isr_handler_add(boiaSensor,InterruptFunction1,(void*) boiaSensor);

/*
    vSemaphoreCreateBinary( xSemaphore_Serial );
        if(xSemaphore_Serial == NULL)
        {
           printf("deu bucho");
        }

*/
        //#endif /* ENABLE_SEMAPHORE_SERIAL  */
  // Cria semafaro binario xSemaphore_ExpIO
    
    


}

/* Configurações de GPIO ESP32 */
void vGpioConf(){
    //rele1 aquecedor
    gpio_config_t GPIOconfig;
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << releAquecedor);
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

    //Sensor temperatura
    GPIOconfig.mode = GPIO_MODE_INPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << tempSensor);
    gpio_config(&GPIOconfig);

    //Sensor Boia
    GPIOconfig.mode = GPIO_MODE_INPUT;
    GPIOconfig.intr_type = GPIO_INTR_ANYEDGE;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (15 << boiaSensor);
    gpio_config(&GPIOconfig);

    //Botão
    GPIOconfig.mode = GPIO_MODE_INPUT;
    GPIOconfig.intr_type = GPIO_INTR_HIGH_LEVEL;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (15 << button);
    gpio_set_level(releBomba,1);
    gpio_config(&GPIOconfig);

}
