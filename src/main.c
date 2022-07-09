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



//thermistor connected to ground
//series resistor connected to 3.3V
#define R0 10000 //thermistor resistance at 25 degrees Celsius 10k
#define th_Coeff 3470 //thermistor coefficient
#define Rseries 10000 // 10K series resistor
#define BUF_SIZE (1024)
#define OFF 0
#define ON 1

void vGpioConf();

//typedef struct {
 //valuer temp

//}tempInfo ;

xQueueHandle tempQueue;
xTaskHandle TaskHandle = NULL;
xTaskHandle TaskHandle2 = NULL;





void vUart_use(float data, int type);

const uart_port_t uart_num = UART_NUM_0;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};




void tempMeasurement(void *pvParameters){
    
    float measuere = 0;

    double rez = 0;
    double kel = 0;
    double cel = 0;

    uint32_t val = 0;
    
    adc1_config_width(ADC_WIDTH_12Bit); // 0 .. 4095
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db); //ADC1 Ch1 = IO37
    
    while(1){
        // for(int i = 0; i < 10; i++){
        //     val += adc1_get_raw(ADC1_CHANNEL_0);
        //     vTaskDelay(2/portTICK_PERIOD_MS);
        // }

        // val = val/10;
        // rez = Rseries/((4095./val) - 1);
        // kel = 1./(1./298.15 + 1./th_Coeff * log(rez/R0));
        // cel = kel - 273.15;
        // printf("ADC: %d, rez: %f Ohm, %f K, %f C\n", val, rez, kel, cel);    
        // printf("tmp celsius: %f", cel);
        // printf("\n rez: %f", rez);
        
        //measuere = cel;
        measuere++;           
        xQueueSend(tempQueue, &measuere, portMAX_DELAY); 
        
        vTaskDelay(pdMS_TO_TICKS(500));

       
        

       // val = 0;
}//while(1)
}//tempMeasurement


void controlAtuadores (void *pvParameters){
    //aquecedor cooler 
    float measuere;

    while (1) {
        xQueueReceive(tempQueue, &measuere, portMAX_DELAY);
        if(measuere > 25.00){
            gpio_set_level(1,1);
            gpio_set_level(16,0);
        }else if (measuere < 23 ){
            gpio_set_level(1,0);
            gpio_set_level(16,1);
        }
        else{
            gpio_set_level(1,0);
            gpio_set_level(16,0);

        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void controlAtuadore2(void *pvParameters){
    float measuere;

    while (1) {
        xQueueReceive(tempQueue, &measuere, portMAX_DELAY);
        if(measuere > 10.00){
            gpio_set_level(1,1);
            gpio_set_level(16,1);
        }else if (measuere < 10.00){
            gpio_set_level(1,0);
            gpio_set_level(16,0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main() {
    
    vGpioConf();
    xTaskCreate(&tempMeasurement, "temperature", configMINIMAL_STACK_SIZE+1024, NULL, 1, &TaskHandle);
    xTaskCreate(&controlAtuadores,"Atuadores",configMINIMAL_STACK_SIZE+1024,NULL,1,&TaskHandle2);
    tempQueue = xQueueCreate(1,sizeof(float *)); 


    


    
}


void vGpioConf(){
    //led
    gpio_config_t GPIOconfig;
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << 1);
    gpio_config(&GPIOconfig);

    //rele
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << 16);
    gpio_config(&GPIOconfig);

    //rele2
    GPIOconfig.mode = GPIO_MODE_OUTPUT;
    GPIOconfig.intr_type = 0;
    GPIOconfig.pull_down_en = 0;
    GPIOconfig.pull_up_en = 0;
    GPIOconfig.pin_bit_mask = (1 << 17);
    gpio_config(&GPIOconfig);


}

/**************** Uart usage **************** */

// float data: O valor a ser exibido.        //

// int type: Seleciona a string padrão onde: //
//                  0 = String de Temperatura.
//                  1 = String de Nível

/**************** Uart usage **************** */


void vUart_use(float data, int type){
   char*  string_sensor1 = malloc(sizeof(char) * BUF_SIZE);
   char*  string_sensor2 = malloc(sizeof(char) * BUF_SIZE);


    switch (type){
    case 0:
        sprintf(string_sensor1," - [Sensor Temperatura: ] %f\n\r",data);
        uart_write_bytes(uart_num,string_sensor1,strlen(string_sensor1));
        break;
    case 1:
        sprintf(string_sensor2," - [Sensor Nivel: ] %f\n\r",data);
        uart_write_bytes(uart_num,string_sensor2,strlen(string_sensor2));
        break;
    default:
        break;
    }

    uart_flush(uart_num);
    uart_flush_input(uart_num);

}