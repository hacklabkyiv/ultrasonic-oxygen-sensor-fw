/**
  ******************************************************************************
  * @file           : sensor.c
  * @brief          : OpenOx Ultrasonic Oxygen Flow and Concentration sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Artem Syntsyn <a.synytsyn@gmail.com>
  * All rights reserved.</center></h2>
  *
  * This file is part of the OpenOx project
  * https://github.com/hacklabkyiv/openox-sensor
  * Copyright (c) 2015 Artem Synytsyn <a.synytsyn@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, version 3.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  * General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <math.h>
#include "main.h"
#include "sensor.h"

static uint16_t adcData[ADC_DATA_WINDOW_SIZE] = { 0 };
static volatile uint32_t pwmCounter = 0;
const int num_of_pulses = 9;
const int measurement_shift = 1;

const int sample_start = 765;
const int sample_end = 790;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    pwmCounter++;
    if (pwmCounter == measurement_shift) {
      //HAL_ADC_Start_DMA(&hadc, (uint32_t *) adcData, ADC_DATA_WINDOW_SIZE);
    }
    else if (pwmCounter > num_of_pulses) {
      HAL_TIM_HaltAllPWMs();
      HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_9);
      HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_8);
      pwmCounter = 0;
    }
}

int getTemperature(uint16_t adc_value, float beta, float t_ref)
{
  float T_K = (1.00 / ((1 / t_ref)
          + (1 / beta) * (log(4096.0 / (float) (4096.0 - adc_value) - 1.00))));
  return T_K;
}


float getPhase(uint16_t data[], size_t length)
{
  const int SAMPLE_FREQ = 1000000;
  const int WAVE_FREQ = 40000;
  const float SAMPLING_INTERVAL = 1. / SAMPLE_FREQ;
  const float WAVE_PERIOD = 1. / WAVE_FREQ;
  const float WA = 2 * M_PI * WAVE_FREQ;
  float ai = 0;
  float aq = 0;
  for (int i = 0; i < length; ++i)
  {
    float arg = WA * i * SAMPLING_INTERVAL;
    ai += data[i] * cos(arg);
    aq += data[i] * sin(arg);
  }
  float a = sqrt(ai * ai + aq * aq);
  float phi = atan2(ai / a, aq / a);
  float max_time = fmod((2 * M_PI + M_PI_2 - phi) / WA, WAVE_PERIOD);
  float max_sample = max_time / SAMPLING_INTERVAL;
  printf("Params:\r\n");
  printf("%d %.8f %.8f %.8f\r\n", length, ai, aq, WA);
  printf("Peak: %.8f\r\n", max_time);
  printf("Amplitude: %.8f\r\n", max_sample);
  return max_time;
}

void SENSOR_Init()
{
  printf("Sensor initialization...\r\n");
  HAL_Delay(100);
  int index = 0;
  while (1)
  {
    /* Get data from the first pair PWM1 - ADC0 */
    printf("Sample %d\r\n", index);
    float t_0, t_1, diff;
    HAL_GPIO_ReconfigurePinAsPWM(GPIO_PIN_8);
    HAL_Delay(10);
    HAL_TIM_RunPWMChannelOne();
    HAL_ADC_SetChannelZeroActive();
    HAL_Delay(ADC_CONVERTION_TIME);
#if 1
    printf("Channel0-start\r\n");
    for (int i = sample_start; i < sample_end; i++)
      printf("%u,%u\r\n", i, adcData[i]);
    printf("Channel0-completed\r\n");
#endif
    t_0 = getPhase(&adcData[sample_start], sample_end - sample_start);
    /* Get data from the second pair PWM2 - ADC1 */
    HAL_Delay(DELAY_BETWEEN_MEASUREMENTS);
    HAL_GPIO_ReconfigurePinAsPWM(GPIO_PIN_9);
    HAL_Delay(10);
    HAL_TIM_RunPWMChannelTwo();
    HAL_ADC_SetChannelOneActive();
    HAL_Delay(ADC_CONVERTION_TIME);
#if 1
    printf("Channel1-start\r\n");
    for (int i = sample_start; i < sample_end; i++)
      printf("%u,%u\r\n", i, adcData[i]);
    printf("Channel1-completed\r\n");
#endif
    t_1 = getPhase(&adcData[sample_start], sample_end - sample_start);
    HAL_Delay(DELAY_BETWEEN_MEASUREMENTS);
    /* Get temperature from NTC */
    uint16_t adc_temp = 0;
    uint16_t temperature = 0;
    HAL_ADC_SetChannelFourActive();
    //HAL_ADC_Start(&hadc);
    HAL_Delay(ADC_CONVERTION_TIME);
    //adc_temp = HAL_ADC_GetValue(&hadc);
    //HAL_ADC_Stop(&hadc);
    temperature = getTemperature(adc_temp, B_25_50, T_ref);
    printf("Temperature: %u\r\n", temperature);
    /* Calculations */
    diff = t_0 - t_1;
    float flow;
    flow = 0.0016f * diff / (t_0 * t_1);
#if 0
    printf("-------- Calculations ------------\r\n");
    printf("Time Forward: %.8f\r\n", t_0);
    printf("Time Back: %.8f\r\n", t_1);
    printf("Difference: %.8f\r\n", diff);
    printf("Flow: %f\r\n", flow);
#endif
    HAL_Delay(DELAY_BETWEEN_MEASUREMENTS);
  }
}


