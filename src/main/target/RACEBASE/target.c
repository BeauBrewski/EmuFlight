/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2, CH1, PA0, TIM_USE_PPM,   TIMER_INPUT_ENABLED),
    DEF_TIM(TIM8, CH1, PB6, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PWM1 - DMA2 CH3
    DEF_TIM(TIM3, CH4, PB7, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PWM2 - DMA1 CH3
    DEF_TIM(TIM8, CH2, PB8, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PMW3 - DMA2 CH5
    DEF_TIM(TIM17,CH1,PB9, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ),  // PWM4  -DMA1 CH1
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED,   TIMER_OUTPUT_ENABLED ), // LED  - DMA1 CH2
};
//other working setup with dissabled uart1 TX / RX DMA

    //DEF_TIM(TIM8, CH1, PB6, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PWM1 - DMA2 CH3
    //DEF_TIM(TIM4, CH2, PB7, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PWM2 - DMA1 CH4
    //DEF_TIM(TIM4, CH3, PB8, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ), // PMW3 - DMA1 CH5
    //DEF_TIM(TIM17,CH1,PB9, TIM_USE_MOTOR, TIMER_OUTPUT_ENABLED ),  // PWM4  -DMA1 CH1
    //DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED,   TIMER_OUTPUT_ENABLED ), // LED  - DMA1 CH2
