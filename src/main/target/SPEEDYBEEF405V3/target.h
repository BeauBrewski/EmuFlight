/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SBF4"
#define USBD_PRODUCT_STRING  "SPEEDYBEEF405V3"

#define USE_TARGET_CONFIG

#define LED0_PIN                PC8

#define USE_BEEPER
#define BEEPER_PIN              PC5
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_CAMERA_CONTROL
#define CAMERA_CONTROL_PIN      PB3

#define USE_LED_STRIP
#define LED_STRIP_PIN         PC9

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_SPI_GYRO
#define USE_ACC
#define USE_ACCGYRO_BMI270
#define USE_ACC_SPI_MPU6000

#define BMI270_CS_PIN           PA4
#define MPU6000_CS_PIN          PA4
#define BMI270_SPI_INSTANCE     SPI1
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_EXTI
#define BMI270_EXTI_PIN  PC4
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define ACC_BMI270_ALIGN        CW0_DEG
#define GYRO_BMI270_ALIGN       CW0_DEG
#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

// *************** Baro **************************
#define USE_I2C

#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                PB10        // SCL pad
#define I2C2_SDA                PB11        // SDA pad

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define MAG_I2C_INSTANCE           (I2CDEV_2)
#define BARO_I2C_INSTANCE          (I2CDEV_2)

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)


// *************** SD Card **************************
#define USE_SDCARD
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SDCARD_SPI_INSTANCE     SPI2
#define SDCARD_SPI_CS_PIN       PA15

// SPI3 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL                  0

// *************** UART *****************************
#define USE_VCP
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define USE_SOFTSERIAL1
//#define SOFTSERIAL1_RX_PIN      PC12    // Backdoor timer on UART5_RX, used for ESC telemetry
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
//#define SERIALRX_UART           SERIAL_PORT_USART2
#define USE_RX_MSP
#define USE_DSHOT
#define USE_DASHBOARD
#define USE_I2C_OLED_DISPLAY

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE              ADC1
#define ADC1_DMA_STREAM         DMA2_Stream0

#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC2

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 386

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PA3  //PPM
#define ESC_SENSOR_UART         SERIAL_PORT_UART5  //ESC TELEMETRY ON UART5 RX

// *************** activate/deactivate Bluetooth When disarmed/armed using PINIO_BOX *****************************
#define USE_PINIO
#define PINIO1_PIN PC3
#define USE_PINIOBOX

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(5)|TIM_N(8))
