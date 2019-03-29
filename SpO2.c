#include "driverlib.h"
#include <msp430.h>
#include <SpO2.h>

#define __PLETH_GRAPH 10                            // Displays Pleth-Graph
int first_reading=1;

/*******************************************************************************************
 * Lookup Table for spo2
 *******************************************************************************************/
const uint8_t uch_spo2_table[184] = { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
              3, 2, 1 } ;


/*********************************************************************
 * AFE44xx_PowerOn_Init
 * -> Initialize GPIO and SPI communication
 * -> Func Parameter: void
 * -> Return Type: void
 *********************************************************************/
void AFE44xx_PowerOn_Init(void)
{
    volatile unsigned short Init_i, j;
    Init_AFE44xx_Resource();                        // Set GPIO and SPI Pins
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
    }
    Init_AFE44xx_DRDY_Interrupt();                  // Configure Data Ready Interrupt
    afe44xxInit();                                  // Initialize AFE with its Register values
}

/*****************************************************************
 * Init_AFE44xx_Resource
 * -> Configure GPIO and SPI Pins
 * -> Func Parameter: void
 * -> Return Type: void
 *****************************************************************/
void Init_AFE44xx_Resource(void)
{
  Set_GPIO();                                       // Initializes AFE44xx's input control lines
  SPI_init();                                       // Initialize SPI regs.
}

/******************************************
 * Set_GPIO
 * -> Configure GPIO Pins as
 *      P1.2 (PWDN) - Output Pin
 *      P1.3 (START) - Output Pin
 *      P2.4 (PD_ALM) - Input Pin
 *      P2.1 (LED_ALM) - Input Pin
 *      P2.6 (DRDY) - Input Pin
 *      P4.1 (MOSI) - Output Pin
 *      P4.2 (MISO) - Input Pin
 *      P4.3 (SCK) - Output Pin
 *      P4.0 (CS0) - Output Pin
 *      P5.7 (DIAG_END) - Input Pin
 * -> Func Parameter: void
 * -> Return Type: void
 ******************************************/
void Set_GPIO (void)
{
    // Set the Port Pins to Output Direction
    GPIO_setAsOutputPin(PWDN);                                  // P1.2 (If PWDN=0, AFE will go to Power Down Mode)
    GPIO_setAsOutputPin(START);                                 // P1.3 (START)
    GPIO_setAsOutputPin(PD_ALM);                                // P2.4 (PD_ALM)
    GPIO_setAsOutputPin(LED_ALM);                               // P2.1 (LED_ALM)
    GPIO_setAsOutputPin(DRDY);                                  // P2.6 (DRDY)
    GPIO_setAsOutputPin(MOSI);                                  // P4.1 (MOSI)
    GPIO_setAsOutputPin(MISO);                                  // P4.2 (MISO)
    GPIO_setAsOutputPin(SCK);                                   // P4.3 (SCK)
    GPIO_setAsOutputPin(CS0);                                   // P4.0 (CS0)
    GPIO_setAsOutputPin(DIAG_END);                              // P5.7 (DIAG_END)

    // Set all the Output Port Pins low except Port1 and Port2
    GPIO_setOutputHighOnPin(PWDN);                              // Set P1.2 High (If PWDN=0, AFE will go to Power Down Mode)
    GPIO_setOutputHighOnPin(START);                             // Set P1.3 (START) High
    GPIO_setOutputHighOnPin(PD_ALM);                            // Set P2.4 (PD_ALM) High
    GPIO_setOutputHighOnPin(LED_ALM);                           // Set P2.1 (LED_ALM) High
    GPIO_setOutputHighOnPin(DRDY);                              // Set P2.6 (DRDY) High

    GPIO_setOutputLowOnPin(SET_SPI_B1_ALL_4PINS);               // Set P4.0 (CS0), P4.1 (MOSI), P4.2 (MISO), P4.3 (SCK) Low
    GPIO_setOutputLowOnPin(DIAG_END);                           // Set P5.7 (DIAG_END) Low

    // Set P1.2(PWDN) & P1.3(START) to output direction
    GPIO_setAsOutputPin(PWDN);
    GPIO_setAsOutputPin(START);

    // Set P1.2 (PWDN) & P1.3 (START) High (If PWDN=0, AFE will go to Power Down Mode), initially.
    GPIO_setOutputHighOnPin(PWDN);
    GPIO_setOutputHighOnPin(START);

    // Set P2.1(LED_ALM), P2.4(PD_ALM) & P2.6(DRDY) & P5.7 (DIAG_END)to input direction
    GPIO_setAsInputPin(LED_ALM);
    GPIO_setAsInputPin(PD_ALM);
    GPIO_setAsInputPin(DRDY);
    GPIO_setAsInputPin(DIAG_END);
}

/*********************************************************************************
 * SPI_init
 * -> Configure SPI Module: Set GPIO to Peripheral Module Function (i.e. UCB1)
 * -> Func Parameter: void
 * -> Return Type: void
 *********************************************************************************/
void SPI_init(void)
{
    /*  ------------Steps to initialize SPI Module:--------------
     *  1. USCI_B_SPI_initMaster(uint16_t baseAddress, USCI_B_SPI_initMasterParam *param);
     *  2. Enable SPI module using: USCI_B_SPI_enable (uint16_t baseAddress);
     *  3. USCI_B_SPI_disableInterrupt (uint16_t baseAddress, uint8_t mask)
     *  */

     //P4.2,1 & P4.3 option select
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P4,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN3
            );                                                                      // Set Port4 as Peripheral Module Function
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4,
            GPIO_PIN2
            );                                                                      // P4.2-MISO as input

     GPIO_setOutputHighOnPin(CS0);                                                  // Set Chip Select high

     //Initialize Master
     USCI_B_SPI_initMasterParam param = {0};
     USCI_B_SPI_disable(USCI_B1_BASE);                                              // Enable SW reset
     param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
     param.clockSourceFrequency = UCS_getSMCLK();                                   // SMCLK clock frequency
     param.desiredSpiClock = SPICLK;
     param.msbFirst = USCI_B_SPI_MSB_FIRST;                                         // MSB First
     param.clockPhase = USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;     // Clock phase (SPI Mode 1 : CPOL=0 & CPHA=1)
     param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;                 // Clock polarity

     status_initMaster = USCI_B_SPI_initMaster(
                                 USCI_B1_BASE,
                                 &param
                                 );                                                 // Initialize Master

     USCI_B_SPI_enable(USCI_B1_BASE);                                               //  Clear SW reset and Enable SPI module
     USCI_B_SPI_disableInterrupt(
             USCI_B1_BASE,
             USCI_B_SPI_RECEIVE_INTERRUPT + USCI_B_SPI_TRANSMIT_INTERRUPT
             );                                                                     // Disable both Rx and Tx interrupts
     GPIO_setOutputLowOnPin(CS0);
}

/*****************************************************************
 * Init_AFE44xx_DRDY_Interrupt
 * -> Configure Port 2 as Data Ready Interrupt
 * -> Func Parameter: void
 * -> Return Type: void
 *****************************************************************/
void Init_AFE44xx_DRDY_Interrupt (void)
{
    GPIO_setAsInputPinWithPullUpResistor(DRDY);                                      // Enable Input Pin P2.6 as pull-up resistance
    GPIO_selectInterruptEdge(DRDY, GPIO_HIGH_TO_LOW_TRANSITION);                     // High-to-Low Transition
    GPIO_clearInterrupt(DRDY);                                                       // P2.6 IFG cleared
    GPIO_disableInterrupt(DRDY);                                                     // P2.6 interrupt disabled
}

/*****************************************************************
 * Enable_AFE44xx_DRDY_Interrupt
 * -> Clear Interrupt flag & Enable Data Ready Interrupt
 * -> Func Parameter: void
 * -> Return Type: void
 *****************************************************************/
void Enable_AFE44xx_DRDY_Interrupt (void)
{
    GPIO_clearInterrupt(DRDY);                                                       // P2.6 IFG cleared
    GPIO_enableInterrupt(DRDY);                                                      // P2.6 interrupt enabled
}

/*****************************************************************
 * Disable_AFE44xx_DRDY_Interrupt
 * -> Clear Interrupt Flag and Disable Data Ready Interrupt
 * -> Func Parameter: void
 * -> Return Type: void
 *****************************************************************/
void Disable_AFE44xx_DRDY_Interrupt (void)
{
    GPIO_clearInterrupt(DRDY);                                                      // P2.6 IFG cleared
    GPIO_disableInterrupt(DRDY);                                                    // P2.6 interrupt disabled
}

/**********************************************************************************
 * afe44xxInit
 * -> Write data to AFE4490 Registers using SPI Protocol
 * -> Initialize AFE: Sampling of LED's, Start internal ADC
 * -> Func Parameters: void
 * -> Return Type: void
 **********************************************************************************/
void afe44xxInit (void)
{
    Disable_AFE44x0_SPI_Read();                 // Enable SPI Write

    AFE44x0_Reg_Write(CONTROL0,0x000000);
    AFE44x0_Reg_Write(CONTROL0,0x000008);
    AFE44x0_Reg_Write(TIAGAIN,0x000000);        // CF = 5pF, RF = 500kR
    AFE44x0_Reg_Write(TIA_AMB_GAIN,0x000001);
    AFE44x0_Reg_Write(LEDCNTRL,0x001414);
    AFE44x0_Reg_Write(CONTROL2,0x000000);       // LED_RANGE=100mA, LED=50mA
    AFE44x0_Reg_Write(CONTROL1,0x010707);       // Timers ON, average 3 samples
    AFE44x0_Reg_Write(PRPCOUNT, 0X001F3F);      // 7999
    AFE44x0_Reg_Write(LED2STC, 0X001770);
    AFE44x0_Reg_Write(LED2ENDC,0X001F3E);
    AFE44x0_Reg_Write(LED2LEDSTC,0X001770);
    AFE44x0_Reg_Write(LED2LEDENDC,0X001F3F);
    AFE44x0_Reg_Write(ALED2STC, 0X000000);
    AFE44x0_Reg_Write(ALED2ENDC, 0X0007CE);
    AFE44x0_Reg_Write(LED2CONVST,0X000002);
    AFE44x0_Reg_Write(LED2CONVEND, 0X0007CF);
    AFE44x0_Reg_Write(ALED2CONVST, 0X0007D2);
    AFE44x0_Reg_Write(ALED2CONVEND,0X000F9F);
    AFE44x0_Reg_Write(LED1STC, 0X0007D0);
    AFE44x0_Reg_Write(LED1ENDC, 0X000F9E);
    AFE44x0_Reg_Write(LED1LEDSTC,  0X0007D0);
    AFE44x0_Reg_Write(LED1LEDENDC,0X000F9F);
    AFE44x0_Reg_Write(ALED1STC, 0X000FA0);
    AFE44x0_Reg_Write(ALED1ENDC, 0X00176E);
    AFE44x0_Reg_Write(LED1CONVST, 0X000FA2);
    AFE44x0_Reg_Write(LED1CONVEND, 0X00176F);
    AFE44x0_Reg_Write(ALED1CONVST, 0X001772);
    AFE44x0_Reg_Write(ALED1CONVEND, 0X001F3F);
    AFE44x0_Reg_Write(ADCRSTSTCT0, 0X000000);
    AFE44x0_Reg_Write(ADCRSTENDCT0,0X000000);
    AFE44x0_Reg_Write(ADCRSTSTCT1, 0X0007D0);
    AFE44x0_Reg_Write(ADCRSTENDCT1, 0X0007D0);
    AFE44x0_Reg_Write(ADCRSTSTCT2, 0X000FA0);
    AFE44x0_Reg_Write(ADCRSTENDCT2, 0X000FA0);
    AFE44x0_Reg_Write(ADCRSTSTCT3, 0X001770);
    AFE44x0_Reg_Write(ADCRSTENDCT3, 0X001770);

    __delay_cycles(1000);                         // Wait for 1000 MCLK Cycles
    Enable_AFE44x0_SPI_Read();                    // Disable SPI Write
}

/*************************************************************
 * AFE44x0_Reg_Write
 * -> Write Data to AFE Register
 * -> Func Parameters: 1) Address of Register  2)Data to be written
 * -> Return Type: void
 *************************************************************/
void AFE44x0_Reg_Write (unsigned char reg_address, unsigned long data)
{
    unsigned char dummy_rx;

    GPIO_setOutputLowOnPin(CS0);                                          //  Set Chip Select LOW FOR SOT

    // Loop unrolling for machine cycle optimization
    // Send the first byte to the TX Buffer: Address of register
    USCI_B_SPI_transmitData(
            USCI_B1_BASE,
            reg_address
            );
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;           // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);    // Dummy Read Rx buf

    // Send the second byte to the TX Buffer: Data[23:16]
    USCI_B_SPI_transmitData(
            USCI_B1_BASE,
            (unsigned char)(data >>16)
            );
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;            // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);     // Dummy Read Rx buf

    // Send the third byte to the TX Buffer: Data[15:8]
    USCI_B_SPI_transmitData(
            USCI_B1_BASE,
            (unsigned char)((data & 0x00FFFF) >>8)
            );
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;            // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);     // Dummy Read Rx buf

    // Send the last byte to the TX Buffer: Data[7:0]
    USCI_B_SPI_transmitData(
            USCI_B1_BASE,
            (unsigned char)(data & 0x0000FF)
            );
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;            // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);     // Dummy Read Rx buf

    GPIO_setOutputHighOnPin(CS0);    // Set Chip Select HIGH for EOT

}

/**********************************************************************************
 * AFE44x0_Reg_Read
 * -> Read Data from AFE
 * -> Func Parameters: 1) Register Address (from which data is to be read)
 * -> Return Type: Data Read from AFE Register Address
 **********************************************************************************/
unsigned long AFE44x0_Reg_Read(unsigned char reg_address)
{
    unsigned char SPI_Rx_buf[4];
    unsigned long retVal;
    retVal = 0;

    GPIO_setOutputLowOnPin(CS0);                                                  // Set CS0 LOW for SOT
    // Loop unrolling for machine cycle optimization

    // Send the first byte to the TX Buffer: Address of register
    USCI_B_SPI_transmitData(
            USCI_B1_BASE,
            reg_address
            );
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    SPI_Rx_buf[0] = USCI_B_SPI_receiveData(USCI_B1_BASE);       // Read Rx buf

    // Send the second byte to the TX Buffer: dummy data
    USCI_B_SPI_transmitData(USCI_B1_BASE,0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    SPI_Rx_buf[1] = USCI_B_SPI_receiveData(USCI_B1_BASE);       // Read Rx buf: Data[23:16]

    // Send the third byte to the TX Buffer: dummy data
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    SPI_Rx_buf[2] = USCI_B_SPI_receiveData(USCI_B1_BASE);       // Read Rx buf: Data[15:8]

    // Send the last byte to the TX Buffer: dummy data
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    SPI_Rx_buf[3] = USCI_B_SPI_receiveData(USCI_B1_BASE);       // Read Rx buf: Data[7:0]

    GPIO_setOutputHighOnPin(CS0);           // set HIGH at end of transmission

    retVal = SPI_Rx_buf[1];
    retVal = (retVal << 8) | SPI_Rx_buf[2];
    retVal = (retVal << 8) | SPI_Rx_buf[3];

    return    retVal;                                           // Return 32bit data
}

/***********************************************************************
 * Enable_AFE44x0_SPI_Read
 * -> Enable SPI_Read Operation
 * -> Sets SPI_Read Bit in CONTROL0 of AFE4490
 * -> Func Parameters: void
 * -> Return Type: void
 ***********************************************************************/
void Enable_AFE44x0_SPI_Read (void)
{
    unsigned char dummy_rx;
    //Set Control0 - Enable SPI Read bit

    GPIO_setOutputLowOnPin(CS0);                                                // CS LOW for start of transmission.
    // Loop unrolling for machine cycle optimization
    // Send the first byte to the TX Buffer: Address of register
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the second byte to the TX Buffer: Data[23:16]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the third byte to the TX Buffer: Data[15:08]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the last byte to the TX Buffer: Data[7:0]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 1);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    GPIO_setOutputHighOnPin(CS0);                                                  // CS High for end of transmission
}

/***********************************************************************
 * Disable_AFE44x0_SPI_Read
 * -> Disable SPI_Read Operation
 * -> Clears SPI_Read bit in CONTROL0 of AFE4490
 * -> Func Parameters: void
 * -> Return Type: void
 ***********************************************************************/
void Disable_AFE44x0_SPI_Read (void)
{
    unsigned char dummy_rx;
    //Set Control0 - Disable SPI Read bit

    GPIO_setOutputLowOnPin(CS0);                                                  // CS LOW for start of transmission.
    // Loop unrolling for machine cycle optimization
    // Send the first byte to the TX Buffer: Address of register
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the second byte to the TX Buffer: Data[23:16]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the third byte to the TX Buffer: Data[15:08]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    // Send the last byte to the TX Buffer: Data[7:0]
    USCI_B_SPI_transmitData(USCI_B1_BASE, 0);
    while (USCI_B_SPI_isBusy(USCI_B1_BASE)) ;                   // USCI_B1 TX buffer ready?
    dummy_rx = USCI_B_SPI_receiveData(USCI_B1_BASE);            // Dummy Read Rx buf

    GPIO_setOutputHighOnPin(CS0);                                                  // CS High for end of transmission
}

/***********************************************************************************************************
 * estimate_spo2
 * -> CALCULATE SPO2 AND PULSE RATE
 * -> Using Peak Detector as Valley Detector and 4-Point Moving Average
 * -> Function Parameters:
 *      1. pun_ir_buffer - IR data
 *      2. n_ir_buffer_length - 100
 *      3. pun_red_buffer - Red data
 *      4. pn_spo2 - spo2 data (%)
 *      5. pch_spo2_valid - Flag indicating true spo2
 *      6. pn_heart_rate - HeartRate data
 *      7. pch_hr_valid - Flag indicating true HeartRate
 * -> Return Type: void
 ***********************************************************************************************************/
void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  uint32_t un_ir_mean;
  int32_t k, n_i_ratio_count;
  int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks;
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;
  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx, n_x_dc_max_idx;
  int32_t an_ratio[5], n_ratio_average;
  int32_t n_nume, n_denom ;

  // calculates DC mean and subtract DC from ir
  un_ir_mean = 0;

  for(k=0; k < n_ir_buffer_length; k++ )
  {
      un_ir_mean += pun_ir_buffer[k] ;
  }
  un_ir_mean = un_ir_mean/n_ir_buffer_length ; // Average of IR values

  // remove DC and invert signal so that we can use peak detector as valley detector
  for(k=0; k < n_ir_buffer_length; k++ )
      an_x[k] = -1*(pun_ir_buffer[k] - un_ir_mean) ; // The detection of valleys instead of peaks is performed simply by negating the data

  // 4 pt Moving Average of IR signal (FIR Low Pass Filter)-increases the smoothness of the output, whereas the sharp transitions in the data are made increasingly blunt
  for(k=0; k < (BUFFER_SIZE-MA4_SIZE); k++)
  {
      an_x[k] = (an_x[k] + an_x[k+1] + an_x[k+2] + an_x[k+3])/(int)4;
  }

  // calculate threshold so as to detect valley greater than that
  n_th1 = 0; // Used as Minimum Height (To Detect peaks that are greater than minimum peak height)
  for( k=0; k < BUFFER_SIZE; k++)
  {
      n_th1 += an_x[k];
  }
  n_th1 = n_th1/BUFFER_SIZE; // Average of threshold

  if(n_th1 < 30)
      n_th1 = 30; // min allowed
  if(n_th1 > 60)
      n_th1 = 60; // max allowed

  for( k=0; k<15; k++)
      an_ir_valley_locs[k] = 0;

  // Since we flipped signal, we use peak detector as valley detector
  find_peak( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 ); // Find peaks above minimum height and Remove peaks separated by less than MIN_DISTANCE
  n_peak_interval_sum = 0;

  if(n_npks >= 2)
  {
      for(k=1; k<n_npks; k++)
          n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k -1]); // Sum of peak intervals between IR valley locations

      n_peak_interval_sum = n_peak_interval_sum/(n_npks-1);
      *pn_heart_rate = (int32_t)((FS*60) / n_peak_interval_sum);
      *pch_hr_valid  = 1;
  }
  else
  {
      *pn_heart_rate = -999;  // Unable to calculate because peaks are too small
      *pch_hr_valid  = 0;
  }

  // Load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for(k=0; k < n_ir_buffer_length; k++) // n_ir_buffer_length = 100
  {
      an_x[k] = pun_ir_buffer[k] ;
      an_y[k] = pun_red_buffer[k] ;
  }

  // Find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count = n_npks;

  //  Using exact_ir_valley_locs , find ir-red DC and ir-red AC for SPO2 calibration an_ratio
  //  Finding AC/DC maximum of raw

  n_ratio_average = 0;
  n_i_ratio_count = 0;

  for(k=0; k<5; k++)
      an_ratio[k] = 0;

  for(k=0; k < (n_exact_ir_valley_locs_count); k++)
  {
      if(an_ir_valley_locs[k] > BUFFER_SIZE)
      {
          *pn_spo2 =  -999 ; // Do not use SPO2 since valley loc is out of range
          *pch_spo2_valid  = 0;
          return;
      }
  }
  // Find max between two valley locations and use an_ratio between AC component of Ir & Red and DC component of Ir & Red for SPO2
  for(k=0; k < (n_exact_ir_valley_locs_count-1); k++)
  {
      n_y_dc_max = -16777216;
      n_x_dc_max = -16777216;

      if((an_ir_valley_locs[k+1]-an_ir_valley_locs[k]) > 3) // Can't do any work if there are less than 3 points to work with
      {
          for(i=an_ir_valley_locs[k]; i < an_ir_valley_locs[k+1]; i++)
          {
              if(an_x[i] > n_x_dc_max)
              {
                  n_x_dc_max = an_x[i]; // Store the greatest value of IR in n_x_dc_max
                  n_x_dc_max_idx = i; // Preserve the index of that value in the array
              }
              if (an_y[i] > n_y_dc_max) // Similarly for Red
              {
                  n_y_dc_max = an_y[i];
                  n_y_dc_max_idx = i;
              }
          }
          n_y_ac = (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k]]) * (n_y_dc_max_idx -an_ir_valley_locs[k]); // Red
          n_y_ac =  an_y[an_ir_valley_locs[k]] + (n_y_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]));
          n_y_ac =  an_y[n_y_dc_max_idx] - n_y_ac;    // Subtracting linear DC components from raw

          n_x_ac = (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_ir_valley_locs[k]); // IR
          n_x_ac =  an_x[an_ir_valley_locs[k]] + (n_x_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]));
          n_x_ac =  an_x[n_y_dc_max_idx] - n_x_ac;   // Subtracting linear DC components from raw

          n_nume = ( n_y_ac * n_x_dc_max )>>7; // Prepare X100 to preserve floating value
          n_denom = ( n_x_ac * n_y_dc_max )>>7;

          if (n_denom>0  && (n_i_ratio_count)<5 && n_nume != 0) // Denominator should not be less than or equal to Zero
          {
              an_ratio[n_i_ratio_count] = (n_nume*100)/n_denom ; // Formula is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
              n_i_ratio_count++;
          }
      }
  }

  // Choose median value since PPG signal may varies from beat to beat
  sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx = n_i_ratio_count/2;

  if(n_middle_idx >1)
      n_ratio_average = (an_ratio[n_middle_idx-1] + an_ratio[n_middle_idx])/2; // Use median
  else
      n_ratio_average = an_ratio[n_middle_idx ];

  if(n_ratio_average>2 && n_ratio_average <184) // Check Lookup table for spo2 with reference to average ratio
  {
      n_spo2_calc = uch_spo2_table[n_ratio_average] ;
      *pn_spo2 = n_spo2_calc ;
      *pch_spo2_valid  = 1; // Float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else
  {
    *pn_spo2 = -999 ; // Do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }
}

/****************************************************************************************************
 * find_peak
 * -> Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
 * -> Func Parameters:
 *          1. pn_locs - Base address of array to Store Location of IR valley
 *          2. n_npks - No. of IR Peaks
 *          3. pn_x - Base address of IR buffer
 *          4. n_size - Buffer size
 *          5. n_min_height - Minimum Height
 *          6. n_min_distance - Minimum Distance
 *          7. n_max_num - Maximum no. of peaks
 * -> Return Type: void
 ****************************************************************************************************/
void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
{
    find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height ); // Find all the peaks above minimum height
    remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance ); // Remove the peaks separated by minimum distance
    *n_npks = min( *n_npks, n_max_num );
}

/************************************************************************************************************
 * find_peak_above
 * -> Find all peaks above MIN_HEIGHT
 * -> Func Parameters:
 *      1. pn_locs - Base address of array to Store Location of IR valley
 *      2. n_npks - No. of IR Peaks
 *      3. pn_x - Base address of IR buffer
 *      4. n_size - Buffer size
 *      5. n_min_height - Minimum Height
 * -> Return Type: void
 ************************************************************************************************************/
void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
{
    int32_t i = 1, n_width;
    *n_npks = 0;

    while(i < n_size-1)
    {
        if(pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1])
        {
            // Find left edge of potential peaks
            n_width = 1;
            while(i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
                n_width++;
            if(pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 )
            {
                // Find right edge of peaks
                pn_locs[(*n_npks)++] = i;
                // For flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

/**************************************************************************************************************************
 * remove_close_peaks
 * -> Detect peaks that are at least separated by minimum peak distance(eg. 5), in number of channels
 * -> Func Parameters:
 *      1. pn_locs - Base address of array to Store Location of IR valley
 *      2. pn_npks - No of IR peaks above minimum height
 *      3. pn_x - Base address of IR buffer
 *      4. n_min_distance - Minimum separation between two peaks
 * -> Return Type: void
 **************************************************************************************************************************/
void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{
    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for(i = -1; i < (*pn_npks); i++ )
    {
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for(j = i+1; j < (n_old_npks); j++)
        {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorrect is at index -1
            if( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }
    // Resort indices int32_to ascending order
    sort_ascend( pn_locs, *pn_npks );
}

/*********************************************************************************
 * sort_ascend
 * -> Sort array in ascending order (insertion sort algorithm)
 * -> Func Parameters:
 *      1. pn_x - Base address of IR buffer
 *      2. n_size - Buffer Size
 * -> Return Type: void
 *********************************************************************************/
void sort_ascend(int32_t  *pn_x, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

/****************************************************************************************************
 * sort_indices_descend
 * -> Sort indices according to descending order (insertion sort algorithm)
 * -> Func Parameters:
 *      1. pn_x - Base address of IR buffer
 *      2. pn_indx - Address of array to Store Location of IR valley
 *      3. n_size - Buffer Size
 * -> Return Type: void
 ****************************************************************************************************/
void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}

/***********************************************************************************
 * clock_config_16
 * -> Set Clock Frequency to 16MHz
 * -> Configure UCS Module (See Userguide for MSP430F5529)
 * -> Func Parameter: void
 * -> Return Type: void
 ***********************************************************************************/
void clock_config_16(void)
{
    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx

    // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
     }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

     __bis_SR_register(SCG0);                  // Disable the FLL control loop
     UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
     UCSCTL2 |= 450;                           // Set DCO Multiplier for 8MHz
                                               // (N + 1) * FLLRef = Fdco
                                               // (249 + 1) * 32768 = 8MHz
     __bic_SR_register(SCG0);                  // Enable the FLL control loop

     // Worst-case settling time for the DCO when the DCO range bits have been
     // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     // UG for optimization.
     // 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
     __delay_cycles(250000);
}

/****************************************************************************
 * SpO2_init
 * -> Initialize SpO2 Module: Configure GPIO, SPI and Data Ready Interrupt
 * -> Func Parameter: void
 * -> Return Type: void
 ****************************************************************************/
void SpO2_init (void)
{
    SetVCore(3);                                        // Power Management Module
    clock_config_16();                                  // Configure Clock to 16MHz
    AFE44xx_PowerOn_Init();                             // Configure GPIO, UCB1 as SPI, and Data Ready Interrupt
    uart_config();                                      // Configure UART settings at Baud Rate as 115200

    __enable_interrupt();                               // Enable Interrupt
    __delay_cycles(1000);                               // 1000 MCLK Cycles for enabling interrupts
}

/**********************************************************************
 * Start SpO2
 * -> Read RED and IR Data from AFE, Estimate SpO2 and Heart Rate.
 * -> for Displaying the Pleth-Signal: Define Macro __PLETH_GRAPH
 * -> for Displaying SpO2 and Heart Rate: Remove Macro __PLETH_GRAPH
 * -> Func Parameters: void
 * -> Return Type: void
 **********************************************************************/
void Start_SpO2(void)
{
              while(1)
            {

                    if (DRDY_trigger == 1)                        // Sets DRDY_trigger if ISR of Port_2 is served
                    {
                            Disable_AFE44xx_DRDY_Interrupt();
                            Enable_AFE44x0_SPI_Read();            // Enable Reading by setting SPI_Read in CONTROL0
                            IRtemp= AFE44x0_Reg_Read(44);         // Read IR data from LED1VAL(address: 0x2C)
                            Enable_AFE44x0_SPI_Read();
                            REDtemp = AFE44x0_Reg_Read(42);       // Read Red data from LED2VAL(address: 0x2A)
                            afe44xx_data_ready = true;
#ifdef __PLETH_GRAPH
                            uart_data_send(IRtemp);
                            uart_string("\r\n");
#endif
#ifndef __PLETH_GRAPH
                            if(IRtemp == 2096921)                 // Condition for Finger not present in SpO2 Probe
                            {
                                uart_string("Insert Finger");
                                uart_string("\r\n");
                            }
#endif
                    }

                    if(afe44xx_data_ready == true)
                    {
                            IRtemp = (unsigned long) (IRtemp<<10);
                            seegtemp = (signed long) (IRtemp);
                            seegtemp = (signed long) (seegtemp>>10);

                            REDtemp = (unsigned long) (REDtemp<<10);
                            seegtemp2 = (signed long) (REDtemp);
                            seegtemp2 = (signed long) (seegtemp2>>10);

                            if(dec==20)
                            {
                                    aun_ir_buffer[n_buffer_count] = (uint16_t) (seegtemp>>4);
                                    aun_red_buffer[n_buffer_count] = (uint16_t) (seegtemp2>>4);
                                    n_buffer_count++;
                                    dec = 0;
                            }
                            dec++;

                            if(n_buffer_count>99)
                            {
                                    estimate_spo2(aun_ir_buffer, 100, aun_red_buffer, &n_spo2, &ch_spo2_valid,&n_heart_rate, &ch_hr_valid); // Process the data and Calculate spo2
                                    if(n_spo2 == -999)            // Invalid data
                                    {
#ifndef __PLETH_GRAPH
                                        uart_string("Probe error!!!!");
                                        uart_string("\r\n");
#endif
                                    }
                                    else                          // Valid data
                                    {
                                        HR_avg(n_heart_rate);
#ifndef __PLETH_GRAPH
                                        uart_string("\r\n");
                                        uart_string("calculating sp02...");
                                        uart_string("\r\n");

                                        uart_string("Sp02 : ");
                                        uart_data_send(n_spo2);
                                        uart_string("%");
                                        uart_string("\r\n");

                                        uart_string("Pulse rate :");
                                        uart_data_send(Pulse_Rate_previous);
                                        uart_string("\r\n");
#endif
                                    }
                                    n_buffer_count = 0;
                            }
                            afe44xx_data_ready = false;
                            DRDY_trigger = 0;
                    }
                    Enable_AFE44xx_DRDY_Interrupt();
            }
}

/**************************************************************
 * Stop_SpO2
 * -> Release and disable all the GPIO and Interrupts
 * -> Func Parameters: void
 * -> Return Type: void
 **************************************************************/
void Stop_SpO2 (void)
{
    Disable_AFE44xx_DRDY_Interrupt();                   // Disable Data Ready interrupt
    Clear_All_SpO2_PortPins();                          // Reset All the Port Pins (Output Directed & Low)
}

/**************************************************************
 * Clear_All_SpO2_PortPins
 * -> Direct all the Ports to Output
 * -> Set Low on the Pins
 * -> Func Parameters: void
 * -> Return Type: void
 **************************************************************/
void Clear_All_SpO2_PortPins (void)
{
    // Set as Output
    GPIO_setAsOutputPin(PWDN);                                  // P1.2 (If PWDN=0, AFE will go to Power Down Mode)
    GPIO_setAsOutputPin(START);                                 // P1.3 (START)
    GPIO_setAsOutputPin(PD_ALM);                                // P2.4 (PD_ALM)
    GPIO_setAsOutputPin(LED_ALM);                               // P2.1 (LED_ALM)
    GPIO_setAsOutputPin(DRDY);                                  // P2.6 (DRDY)
    GPIO_setAsOutputPin(MOSI);                                  // P4.1 (MOSI)
    GPIO_setAsOutputPin(MISO);                                  // P4.2 (MISO)
    GPIO_setAsOutputPin(SCK);                                   // P4.3 (SCK)
    GPIO_setAsOutputPin(CS0);                                   // P4.0 (CS0)
    GPIO_setAsOutputPin(DIAG_END);                              // P5.7 (DIAG_END)

    // Clear all the Ports used
    GPIO_setOutputLowOnPin(PWDN);                                  // P1.2 (If PWDN=0, AFE will go to Power Down Mode) = 0
    GPIO_setOutputLowOnPin(START);                                 // P1.3 (START) = 0
    GPIO_setOutputLowOnPin(PD_ALM);                                // P2.4 (PD_ALM) = 0
    GPIO_setOutputLowOnPin(LED_ALM);                               // P2.1 (LED_ALM) = 0
    GPIO_setOutputLowOnPin(DRDY);                                  // P2.6 (DRDY) = 0
    GPIO_setOutputLowOnPin(MOSI);                                  // P4.1 (MOSI) = 0
    GPIO_setOutputLowOnPin(MISO);                                  // P4.2 (MISO) = 0
    GPIO_setOutputLowOnPin(SCK);                                   // P4.3 (SCK) = 0
    GPIO_setOutputLowOnPin(CS0);                                   // P4.0 (CS0) = 0
    GPIO_setOutputLowOnPin(DIAG_END);                              // P5.7 (DIAG_END) = 0
}

/*********************************************************************
* HR_avg
* -> Heart Rate Averaging
* -> Function Parameters: Pulse Rate
* -> Return Type: void
**********************************************************************/
void HR_avg(unsigned char Pulse_Rate)
{
    unsigned char hr_temp;
    hr_temp = 0;
    hr_temp = Pulse_Rate;
    if (first_reading==1)
    {
        Pulse_Rate_previous = hr_temp;
        first_reading = 2;
    }
    else if (first_reading==2)
    {
        if (hr_temp > Pulse_Rate_previous)
        {
            hr_temp = Pulse_Rate_previous + 1;
            Pulse_Rate_previous = hr_temp;
        }
        else if (hr_temp < Pulse_Rate_previous)
        {
            hr_temp = Pulse_Rate_previous - 1;
            Pulse_Rate_previous = hr_temp;
        }
    }

}


/*************************************************************************
 * Data Ready Interrupt Service Routine (P2.6)
 * -> Clear the Interrupt
 * -> Set drdy_trigger flag to indicate Data as ready to be read from AFE
 *************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    switch (P2IV)
    {
        case P2IV_P2IFG6:
            GPIO_clearInterrupt(DRDY);                                   // Clear P2.6 IFG i.e Data RDY interrupt status
            data = 1;
            drdy_trigger = 1;                                            // Set Flag to read AFE44x0 ADC REG data
            break;

        case P2IV_NONE:
            break;
    }
}

//------------------------UART.c----------------------------

void uart_tx(unsigned char data)
{
    UCA0TXBUF = data;
    while (!(UCA0IFG & UCTXIFG));
    UCA0IFG &= (~UCTXIFG);
}

char uart_rx()
{
    while (!(UCA0IFG & UCRXIFG));
    UCA0IFG &= ~(UCRXIFG);
    return UCA0RXBUF;
}

void receive_string()
{
    int i = 0;
    while((ch = uart_rx()))
    {
        rx_data[i++] = ch;
        if(ch == 10)
        break;
    }
    rx_data[--i] = '\0';
}

void send_string(unsigned char * str)
{
    while(*str)
    {
        uart_tx(*str);
        str++;
    }
}

unsigned long reverse(unsigned long data)
{
    unsigned int temp_data = 0;
    uart_data = 0;
    if(data == 0)
        num_digit++;
    while(data)
    {
        temp_data = data % 10;
        uart_data = (uart_data * 10) + temp_data;
        num_digit++;
        data = data/10;
    }
    return uart_data;
}

void send_data(unsigned long data)
{
    unsigned long temp_data = 0;
    unsigned char dummy = 0;
    temp_data = reverse(data);
    while(temp_data)
    {
        uart_tx((temp_data%10)+48);
        temp_data = temp_data/10;
        num_digit--;
    }
    while(num_digit)
    {
        uart_tx('0');
        num_digit--;
    }
    num_digit=0;
}

void UART_strcpy(char *wholeString, char *rx_data)
{
    unsigned int i = 0;
    while(rx_data[i])
    {
        wholeString[i]=rx_data[i];
        i++;
    }
    wholeString[i]=rx_data[i];
}

void uart_config(void)
{
     P3SEL = BIT3+BIT4;
     UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
     UCA0CTL1 |= UCSSEL_2;                     // CLK = ACLK
     UCA0BR0 = 138;                            // 32kHz/9600=3.41 (see User's Guide)
     UCA0BR1 = 0x00;
     UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
     UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}


void send_float_data(double data)
{
    unsigned long integer,i=5;
    double fraction;
    integer = data;
    send_data(integer);
    uart_tx('.');
    fraction = data - (double)integer;
    while(i--)
    {
        integer = fraction * 10;
        send_data(integer);
        fraction = (fraction*10) - integer;
    }
}
