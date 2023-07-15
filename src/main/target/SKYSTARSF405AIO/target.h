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

#pragma once

#define TARGET_BOARD_IDENTIFIER "SKST"
#define USBD_PRODUCT_STRING "SKYSTARSF405AIO"

#define USE_GYRO
#define USE_ACC
//#define USE_ACCGYRO_BMI270
//#define USE_BARO_SPI_BMP280
#define USE_FLASHFS
#define USE_FLASH_W25Q128FV
//#define USE_MAX7456
#define USE_BARO
#define USE_BARO_BMP280
#define USE_ADC
#define USE_VCP
#define USE_FLASHFS
#define USE_OSD

//manual
#define RX_PPM_PIN           PA8
//#define BMP280_CS_PIN           PB13
//#define BARO_CS_PIN             PC5
//#define BARO_SPI_INSTANCE     SPI2
//#define DEFAULT_BARO_SPI_BMP280
//

#define USE_LED
#define LED0_PIN             PC14
//#define LED1_PIN             PC15
#define LED_STRIP_PIN        PC8
#define USE_BEEPER
#define BEEPER_PIN           PC13
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN   PC9

#define USE_SPI
#define USE_SPI_DEVICE_1  //gyro
#define SPI1_SCK_PIN         PA5
#define SPI1_MOSI_PIN        PA6
#define SPI1_MISO_PIN        PA7
//#define USE_SPI_DEVICE_2  //baro/max7456
//#define SPI2_SCK_PIN         PB13
//#define SPI2_MOSI_PIN        PB14
//#define SPI2_MISO_PIN        PB15
#define USE_SPI_DEVICE_3  //flash
#define SPI3_SCK_PIN         PC10
#define SPI3_MOSI_PIN        PC11
#define SPI3_MISO_PIN        PB5

#define USE_EXTI
//#define USE_GYRO_EXTI

//manual
#define USE_SPI_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000

//#define GYRO_1_EXTI_PIN      PC4  // should work - src/main/sensors/gyro.c:684
#define MPU_INT_EXTI         PB13  // duplicate should work  - gyro.c:686
//#define GYRO_1_CS_PIN        PA4  // should work - ./src/main/pg/bus_spi.c:73
//#define BMI270_CS_PIN        PA4  // duplicate should work - ./src/main/pg/bus_spi.c:103
//#define BMI270_INT_EXTI     PC4 //future
//#define GYRO_1_SPI_INSTANCE SPI1
//#define GYRO_1_ALIGN        CW180_DEG
//#define ACC_1_ALIGN         CW180_DEG
#define USE_FLASH_W25M512 //testing
#define USE_FLASH_W25     //testing
#define USE_FLASH_W25M    //testing
#define USE_FLASH_M25P16  //testing
//

//#define ACC_BMI270_ALIGN          CW180_DEG
//#define GYRO_BMI270_ALIGN         CW180_DEG
//#define BMI270_CS_PIN             PA4
//#define BMI270_SPI_INSTANCE       SPI1

#define USE_MPU_DATA_READY_SIGNAL
#define ACC_MPU6000_ALIGN         CW180_DEG
#define GYRO_MPU6000_ALIGN        CW180_DEG
#define MPU6000_CS_PIN            PB12
#define MPU6000_SPI_INSTANCE      SPI1
#define GYRO_MPU6000_ALIGN CW180_DEG
#define ACC_MPU6000_ALIGN CW180_DEG



// notice - this file was programmatically generated and may need GYRO_2 manually added.

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE        (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9
// notice - this file was programmatically generated and likely needs MAG/BARO manually added and/or verified.

#define FLASH_CS_PIN         PA15
#define FLASH_SPI_INSTANCE SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

//#define MAX7456_SPI_CS_PIN   PB12
//#define MAX7456_SPI_INSTANCE SPI2

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART3   PC3
#define SERIAL_PORT_COUNT 7
// notice - UART/USART were programmatically generated - should verify UART/USART.
// notice - may need "#define SERIALRX_UART SERIAL_PORT_USART_"
// notice - may need "#define DEFAULT_RX_FEATURE, SERIALRX_PROVIDER
// notice - should verify serial count.

#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN PC2
//#define ADC2_DMA_STREAM DMA2_Stream0 // notice - DMA2_Stream0 likely need correcting, please modify.

//manual
#define ADC_INSTANCE ADC2
//


// notice - this file was programmatically generated and may not have accounted for any source instance of "#define TLM_INVERTED ON", etc.

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
// notice - masks were programmatically generated - must verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL
// notice - incomplete; may need additional DEFAULT_FEATURES; e.g. FEATURE_SOFTSERIAL | FEATURE_RX_SPI

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
// notice - incomplete. add/remove/replace x

// notice - this file was programmatically generated and may be incomplete.
