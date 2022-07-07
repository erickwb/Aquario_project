#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <machine/ieeefp.h>

//thermistor connected to ground
//series resistor connected to 3.3V
#define R0 5000 //thermistor resistance at 25 degrees Celsius
#define th_Coeff 3470 //thermistor coefficient
#define Rseries 10000 // 10K series resistor

void tempMeasurement(void *pvParameters){

double rez = 0;
double kel = 0;
double cel = 0;

uint32_t val = 0;

adc1_config_width(ADC_WIDTH_12Bit); // 0 .. 4095
adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_11db); //ADC1 Ch1 = IO37

while(1){
    for(int i = 0; i < 10; i++){
        val += adc1_get_raw(ADC1_CHANNEL_1);
        vTaskDelay(2/portTICK_PERIOD_MS);
    }

    val = val/10;
    rez = Rseries/((4095./val) - 1);
    kel = 1./(1./298.15 + 1./th_Coeff * log(rez/R0));
    cel = kel - 273.15;
    printf("ADC: %d, rez: %f Ohm, %f K, %f C\n", val, rez, kel, cel);           vTaskDelay(1000/portTICK_PERIOD_MS);
    val = 0;
}//while(1)
}//tempMeasurement

void app_main() {
xTaskCreate(&tempMeasurement, "temperature", 2048, NULL, 2, NULL);
}