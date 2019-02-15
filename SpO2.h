/*
 * Spo.h
 *
 * SpO2 Library for MSP430F5529
 */

#ifndef SPO2_H_
#define SPO2_H_


#include <msp430.h>
#include "device.h"
#include <driverlib.h>
#include "device.h"
#include <stdint.h>
#include "types.h"
#include "gpio.h"
#include "stdint.h"
#include "UART.h"
#include "HAL_MACROS.h"
#include "HAL_UCS.h"
#include "HAL_PMM.h"


#define DRDY                        GPIO_PORT_P2, GPIO_PIN6
#define START                       GPIO_PORT_P1, GPIO_PIN3
#define PWDN                        GPIO_PORT_P1, GPIO_PIN2
#define DIAG_END                    GPIO_PORT_P5, GPIO_PIN7
#define LED_ALM                     GPIO_PORT_P2, GPIO_PIN1
#define PD_ALM                      GPIO_PORT_P2, GPIO_PIN4
#define CS0                         GPIO_PORT_P4, GPIO_PIN0
#define MOSI                        GPIO_PORT_P4, GPIO_PIN1
#define MISO                        GPIO_PORT_P4, GPIO_PIN2
#define SCK                         GPIO_PORT_P4, GPIO_PIN3

#define SET_SPI_B1_ALL_4PINS        GPIO_PORT_P4, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3
#define ALL_P2_AS_INPUT_PINS        GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN4 + GPIO_PIN6

/*******************************************************************
 * Macros
 *******************************************************************/
#define SPICLK          2000000
#define FS              25        //sampling frequency
#define BUFFER_SIZE     (FS*4)
#define MA4_SIZE        4             // DONOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))
#define false           0
#define true            1
#define DELAY_COUNT     2
/********************************************************************
 * Global Variables
 ********************************************************************/
volatile int drdy_trigger;
volatile int afe44xx_data_ready;
unsigned long IRtemp, REDtemp;
static  int32_t an_x[ BUFFER_SIZE];
static  int32_t an_y[ BUFFER_SIZE];
volatile int8_t n_buffer_count;
int data, j, I, L;
int dec;
signed long seegtemp, seegtemp2;

int32_t n_spo2;                 //SPO2 value
int8_t ch_spo2_valid;           //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate;           //heart rate value
int8_t  ch_hr_valid;            //indicator to show if the heart rate calculation is valid
uint16_t aun_ir_buffer[100];    //infrared LED sensor data
uint16_t aun_red_buffer[100];   //red LED sensor data
int status_initMaster;          // SPI Master Initialization: Success=1, Failure=0

// CONTROL0 - Write Only register
#define CONTROL0        0x00    // Configure AFE software and count timer reset, diagnostics enable, and SPI read functions
// CONTROL1 - Read/Write register
#define CONTROL1        0x1E    // Configures the clock alarm pin, timer, and number of averages
#define TIAGAIN         0x20    // Sets the device Trans-Impedance Amplifier gain mode and feedback resistor and capacitor values
#define TIA_AMB_GAIN    0x21    // Configures the ambient light cancellation amplifier gain, cancellation current, and filter corner frequency
#define LEDCNTRL        0x22    // Set LED current range and the LED1 and LED2 drive current
#define CONTROL2        0x23    // Controls the LED transmitter, crystal, and the AFE, transmitter, and receiver power modes
#define ALARM           0x29

// Read/Write Registers
#define PRPCOUNT        0x1D    // Pulse Repetition Period count
#define LED2STC         0x01    // Start timing value for the LED2 signal sample
#define LED2ENDC        0x02    // End timing value for the LED2 signal sample
#define LED2LEDSTC      0x03    // Start timing value for when the LED2 signal turns on
#define LED2LEDENDC     0x04    // End timing value for when the LED2 signal turns off
#define ALED2STC        0x05    // Start timing value for the ambient LED2 signal sample
#define ALED2ENDC       0x06    // End timing value for the ambient LED2 signal sample
#define LED1STC         0x07    // Start timing value for the LED1 signal sample
#define LED1ENDC        0x08    // End timing value for the LED1 signal sample
#define LED1LEDSTC      0x09    // Start timing value for when the LED1 signal turns on
#define LED1LEDENDC     0x0A    // End timing value for when the LED1 signal turns off
#define ALED1STC        0x0B    // Start timing value for the ambient LED1 signal sample
#define ALED1ENDC       0x0C    // End timing value for the ambient LED1 signal sample
#define LED2CONVST      0x0D    // Start timing value for the LED2 conversion
#define LED2CONVEND     0x0E    // End timing value for the LED2 conversion
#define ALED2CONVST     0x0F    // Start timing value for the ambient LED2 conversion
#define ALED2CONVEND    0x10    // End timing value for the ambient LED2 conversion
#define LED1CONVST      0x11    // Start timing value for the LED1 conversion
#define LED1CONVEND     0x12    // End timing value for the LED1 conversion
#define ALED1CONVST     0x13    // Start timing value for the ambient LED1 conversion
#define ALED1CONVEND    0x14    // End timing value for the ambient LED1 conversion
#define ADCRSTSTCT0     0x15    // Start position of the ADC0 reset conversion signal
#define ADCRSTENDCT0    0x16    // End position of the ADC0 reset conversion signal
#define ADCRSTSTCT1     0x17    // Start position of the ADC1 reset conversion signal
#define ADCRSTENDCT1    0x18    // End position of the ADC1 reset conversion signal
#define ADCRSTSTCT2     0x19    // Start position of the ADC2 reset conversion signal
#define ADCRSTENDCT2    0x1A    // End position of the ADC2 reset conversion signal
#define ADCRSTSTCT3     0x1B    // Start position of the ADC3 reset conversion signal
#define ADCRSTENDCT3    0x1C    // End position of the ADC3 reset conversion signal

/***************************************************************************************
 * Function Prototypes
 ***************************************************************************************/
void clock_config_16(void);
void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num );
void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height );
void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void sort_ascend(int32_t  *pn_x, int32_t n_size);
void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);
void Init_AFE44xx_Resource(void);
void afe44xxInit (void);
void AFE44x0_Reg_Write (unsigned char reg_address, unsigned long data);
unsigned long AFE44x0_Reg_Read(unsigned char Reg_address);
void Enable_AFE44x0_SPI_Read (void);
void Disable_AFE44x0_SPI_Read (void);
void Init_AFE44xx_DRDY_Interrupt (void);
void Enable_AFE44xx_DRDY_Interrupt (void);
void Disable_AFE44xx_DRDY_Interrupt (void);
void SpO2_init (void);
void Start_SpO2(void);
void Stop_SpO2 (void);
void Set_GPIO(void);
void Clear_All_SpO2_PortPins (void);
void SPI_init(void);
void AFE44xx_PowerOn_Init(void);


//--------------Uart---------------
void uart_tx(unsigned char data);
char uart_rx();
void receive_string();
void send_string(unsigned char * str);
unsigned long reverse(unsigned long data);
void send_data(unsigned long data);
void UART_strcpy(char *wholeString, char *rx_data);
void uart_config(void);
void send_float_data(double data);

#endif /* SPO2_H_ */
