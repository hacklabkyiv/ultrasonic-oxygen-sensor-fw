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

#include <math.h>
#include "main.h"
#include "sensor.h"

float ai, aq, phi, a, max_time, max_sample;
float flow = 0;
static uint16_t adcData[ADC_DATA_WINDOW_SIZE] = { 0 };
static volatile uint32_t pwmCounter = 0;
const int num_of_pulses = 14;
const int measurement_shift = 35;

int getTemperature(uint16_t adc_value, float beta, float t_ref)
{
  float T_K = (1.00
      / ((1 / t_ref)
          + (1 / beta) * (log(4096.0 / (float) (4096.0 - adc_value) - 1.00))));
  return T_K;
}

float getPhase(uint16_t data[], size_t length)
{
  length -= 1;
  const int SAMPLE_FREQ = 1000000;
  const int WAVE_FREQ = 40000;
  const float SAMPLING_INTERVAL = 1. / SAMPLE_FREQ;
  const float WAVE_PERIOD = 1. / WAVE_FREQ;
  const float WA = 2 * M_PI * WAVE_FREQ;
  ai = 0;
  aq = 0;
  for (int i = 0; i < length; ++i)
  {
    float arg = WA * i * SAMPLING_INTERVAL;
    ai += data[i] * cos(arg);
    aq += data[i] * sin(arg);
  }
  a = sqrt(ai * ai + aq * aq);
  phi = atan2(ai / a, aq / a);
  max_time = fmod((2 * M_PI + M_PI_2 - phi) / WA, WAVE_PERIOD);
  max_sample = max_time / SAMPLING_INTERVAL;
  //  //printf("Params:\r\n");
  //  //printf("%d %.8f %.8f %.8f\r\n", length, ai, aq, WA);
  //  //printf("Peak: %.8f\r\n", max_time);
  //  //printf("Amplitude: %.8f\r\n", max_sample);
  return max_time;
}

void HAL_GPIO_ReconfigurePinAsInput(uint32_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  HAL_GPIO_DeInit(GPIOA, GPIO_Pin);
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_GPIO_ReconfigurePinAsPWM(uint32_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  HAL_GPIO_DeInit(GPIOA, GPIO_Pin);
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_ADC_ActivateChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = { 0 };
  sConfig.Channel = channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_ADC_DeactivateChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = { 0 };
  sConfig.Channel = channel;
  sConfig.Rank = ADC_RANK_NONE;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_HaltAllPWMs()
{
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_TIM_SET_COUNTER(&htim1, 0);
}
void HAL_TIM_RunPWMChannelOne()
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1->ARR / 2);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
}

void HAL_TIM_RunPWMChannelTwo()
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TIM1->ARR / 2);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
}

uint8_t direction = 0;
const int forward_delay = 10;
const int backward_delay = 10000;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  pwmCounter++;
  if (pwmCounter == measurement_shift)
  {
    if (direction)
      for (int i = 0; i < forward_delay; i++)
        asm("NOP");
    else
      for (int i = 0; i < backward_delay; i++)
        asm("NOP");
    HAL_ADC_Start_DMA(&hadc, (uint32_t *) adcData, ADC_DATA_WINDOW_SIZE);
    HAL_TIM_HaltAllPWMs();
    HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_9);
    HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_8);
    pwmCounter = 0;
  } else if (pwmCounter > num_of_pulses)
  {
    /* Disable pulsing, but timer should be still running */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  }
}
uint16_t temperature = 0;
float t_0, t_1;
int cycle = 0;
void print_end()
{
  uint8_t buffer[] = ">";
  HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer)/sizeof(buffer[0]), HAL_MAX_DELAY);
}
void SENSOR_Init()
{
  while (1)
  {
    cycle++;
    /* Channel 0 measurements */
    /* ADC settings */
    direction = 0;
    HAL_ADC_DeactivateChannel(ADC_CHANNEL_1);
    HAL_ADC_DeactivateChannel(NTC_ADC_CHANNEL);
    HAL_ADC_ActivateChannel(ADC_CHANNEL_0);
    /* PWM Pins settings */
    HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_8);
    HAL_GPIO_ReconfigurePinAsPWM(GPIO_PIN_9);
    /* Enable PWM */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TIM1->ARR / 2);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);;
    HAL_Delay(ADC_CONVERTION_TIME);
    HAL_ADC_Stop_DMA(&hadc);
    t_0 = getPhase(adcData, ADC_DATA_WINDOW_SIZE);
    /* Channel 1 measurements */
    /* ADC settings */
    direction = 1;
    HAL_ADC_DeactivateChannel(ADC_CHANNEL_0);
    HAL_ADC_DeactivateChannel(NTC_ADC_CHANNEL);
    HAL_ADC_ActivateChannel(ADC_CHANNEL_1);
    /* PWM Pins settings */
    HAL_GPIO_ReconfigurePinAsInput(GPIO_PIN_9);
    HAL_GPIO_ReconfigurePinAsPWM(GPIO_PIN_8);
    /* Enable PWM */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1->ARR / 2);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_Delay(ADC_CONVERTION_TIME);
    //for (int i = 0; i < ADC_DATA_WINDOW_SIZE; i++)
    //  print_value(adcData[i]);
    HAL_ADC_Stop_DMA(&hadc);
    t_1 = getPhase(adcData, ADC_DATA_WINDOW_SIZE);
    /* Temperature measurements */
    HAL_ADC_DeactivateChannel(ADC_CHANNEL_0);
    HAL_ADC_DeactivateChannel(ADC_CHANNEL_1);
    HAL_ADC_ActivateChannel(NTC_ADC_CHANNEL);
    uint16_t adc_value = 0;
    HAL_ADC_Start(&hadc);
    HAL_Delay(ADC_CONVERTION_TIME);
    adc_value = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    temperature = getTemperature(adc_value, B_25_50, T_ref);
    /* Calculations */
    float diff;
    diff = t_0 - t_1;
    flow = 0.0016f * diff / (t_0 * t_1);
    //print_value(flow);
    //printf("-------- Calculations ------------\r\n");
    //printf("Time Forward: %.8f\r\n", t_0);
    //printf("Time Back: %.8f\r\n", t_1);
    //printf("Difference: %.8f\r\n", diff);
    //printf("Flow: %f\r\n", flow);
    print_end();
  }
}

