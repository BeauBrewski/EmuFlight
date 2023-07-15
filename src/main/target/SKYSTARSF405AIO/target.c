/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
DEF_TIM(TIM4, CH2, PB7, TIM_USE_MOTOR, 0, 0), //motor 1
DEF_TIM(TIM4, CH1, PB6, TIM_USE_MOTOR, 0, 0), //motor 2
DEF_TIM(TIM3, CH1, PB4, TIM_USE_MOTOR, 0, 0), //motor 3
DEF_TIM(TIM2, CH2, PB3, TIM_USE_MOTOR, 0, 0), //motor 4
DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR, 0, 0), //motor 5
DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 0), //motor 6

DEF_TIM(TIM1, CH2, PA9, TIM_USE_LED, 0, 0), //led
//DEF_TIM(TIM8, CH4, PC9, TIM_USE_ANY, 0, 0), //cam control
//DEF_TIM(TIM1, CH1, PA8, TIM_USE_ANY, 0, 0) //rx_ppm
};


// TIM_USE options:
// TIM_USE_ANY
// TIM_USE_BEEPER
// TIM_USE_LED
// TIM_USE_MOTOR
// TIM_USE_NONE
// TIM_USE_PPM
// TIM_USE_PWM
// TIM_USE_SERVO
// TIM_USE_TRANSPONDER

// config.h resources:
// #define MOTOR1_PIN           PB7
// #define MOTOR2_PIN           PB6
// #define MOTOR3_PIN           PB4
// #define MOTOR4_PIN           PB3
// #define MOTOR5_PIN           PB0
// #define MOTOR6_PIN           PB1
// #define TIMER_PIN_MAPPING \
//     TIMER_PIN_MAP( 0, PB0 , 2,  0) \
//     TIMER_PIN_MAP( 1, PB1 , 2,  0) \
//     TIMER_PIN_MAP( 2, PB3 , 1,  0) \
//     TIMER_PIN_MAP( 3, PB4 , 1,  0) \
//     TIMER_PIN_MAP( 4, PB6 , 1,  0) \
//     TIMER_PIN_MAP( 5, PB7 , 1,  0) \
//     TIMER_PIN_MAP( 6, PC8 , 2,  0) \
//     TIMER_PIN_MAP( 7, PC9 , 2,  0) \
//     TIMER_PIN_MAP( 8, PA8 , 1, -1)

// notice - this file was programmatically generated and may be incomplete.
// recommend converting timers from unified-target; however, unified-targets will be sunsetted.
