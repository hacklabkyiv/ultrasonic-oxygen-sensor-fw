/**
  ******************************************************************************
  * @file           : sensor.h
  * @brief          : OpenOx Ultrasonic Oxygen Flow and Concentration sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Artem Syntsyn <a.synytsyn@gmail.com>
  * All rights reserved.</center></h2>
  *
  * This file is part of the OpenOx project
  * https://github.com/hacklabkyiv/openox-sensor
  * Copyright (c) 2020 Artem Synytsyn <a.synytsyn@gmail.com>
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
#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#define ADC_DATA_WINDOW_SIZE 500
#define ADC_CONVERTION_TIME 300
#define DELAY_BETWEEN_MEASUREMENTS 200

#define NTC_ADC_CHANNEL ADC_CHANNEL_4

/* Thermistor data MF52A1 (https://arduino.ua/files/MF52A1.pdf)*/
#define B_25_50 3950.0f
#define T_ref 298.15f

void SENSOR_Init();

#endif /* INC_SENSOR_H_ */
