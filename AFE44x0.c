#include "device.h"
#include "types.h"               //Basic Type declarations
#include "AFE44x0.h"


#define DELAY_COUNT 2
unsigned long int recv_data;

/*****************************************************************
 * Init_AFE44xx_DRDY_Interrupt
 *****************************************************************/
void Init_AFE44xx_DRDY_Interrupt (void)
{
  P2DIR &= ~AFE_ADC_DRDY;
  P2REN |= AFE_ADC_DRDY;                              // Enable P2.3 internal resistance
  P2OUT |= AFE_ADC_DRDY;                                // Set P2.3 as pull-Up resistance
  P2IES |= AFE_ADC_DRDY;                              // P2.3 Hi/Lo edge
  P2IFG &= ~AFE_ADC_DRDY;                               // P2.3 IFG cleared
  P2IE &= ~AFE_ADC_DRDY;                                // P2.3 interrupt disabled
}

/*****************************************************************
 * Enable_AFE44xx_DRDY_Interrupt() - Enable Data Rady Interrupt
 *****************************************************************/
void Enable_AFE44xx_DRDY_Interrupt (void)
{
  P2IFG &= ~AFE_ADC_DRDY;                               // P2.3 IFG cleared
  P2IE |= AFE_ADC_DRDY;                                 // P2.3 interrupt enabled
}

/*****************************************************************
 * Disable_AFE44xx_DRDY_Interrupt() - Disable Data Rady Interrupt
 *****************************************************************/
void Disable_AFE44xx_DRDY_Interrupt (void)
{
  P2IFG &= ~AFE_ADC_DRDY;                               // P2.3 IFG cleared
  P2IE &= ~AFE_ADC_DRDY;                                // P2.3 interrupt disabled
}

/*****************************************************************
 * Set_GPIO() - Configure GPIO Pins
 *****************************************************************/
void Set_GPIO(void)
{
  //Port 1.1 - AFE_RESETZ, P1.2 - AFE_PDNZ, P2.3 - ADC_RDY, P2.4- PD_ALM, P2.5 - LED_ALM, P5.7 - DIAG_END

  P1SEL |= BIT0;
  P1DIR |= BIT0;
  P1OUT |= BIT0;

  P1DIR |= (AFE_RESETZ + AFE_PDNZ);
  P1OUT |= (AFE_RESETZ + AFE_PDNZ);
  P2DIR &= ~(AFE_ADC_DRDY + AFE_PD_ALM + AFE_LED_ALM);
  P5DIR &= ~AFE_DIAG_END;

  P2SEL = 0x00;
  P2DIR &= BIT0;
  P2OUT |= (BIT1 + BIT2 + BIT7);
  P2DIR |= (BIT1 | BIT2 | BIT7);
}

/*****************************************************************
 * Set_UCB1_SPI() - SPI Configuration
 *****************************************************************/
void Set_UCB1_SPI(void)
{
  P4SEL |= BIT1+BIT2+BIT3;          // Set SPI peripheral bits
  P4DIR |= BIT0+BIT1+BIT3;          // STE, SCLK, and DOUT as output
  P4DIR &= ~BIT2;                   // Din as input
  P4OUT |=BIT0;                     // Set STE high
  UCB1CTL1 |= UCSWRST;                      // Enable SW reset
  UCB1CTL0 |= UCMSB+UCCKPH+UCMST+UCSYNC;    // [b0]   1 -  Synchronous mode
  // [b2-1] 00-  3-pin SPI
  // [b3]   1 -  Master mode
  // [b4]   0 - 8-bit data
  // [b5]   1 - MSB first
  // [b6]   0 - Clock polarity high.
  // [b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

  UCB1CTL1 |= UCSSEL_2;                 // SMCLK
  UCB1BR0 = 0x08;                       // 16 MHz
  UCB1BR1 = 0;
  UCB1CTL1 &= ~UCSWRST;                 // Clear SW reset, resume operation
  UCB1IE =0x0;

  //P4OUT &= ~BIT0;             // Set STE low
}

/*****************************************************************
 * Init_AFE44xx_Resource() - Configure GPIO and SPI Pins
 *****************************************************************/
void Init_AFE44xx_Resource(void)
{
  Set_GPIO();                                       // Initializes AFE44xx's input control lines
  Set_UCB1_SPI();                                   // Initialize SPI regs.
}

/**********************************************************************************
 * afe44xxInit() - Write the data to AFE4490 Registers using SPI Protocol
 **********************************************************************************/
void afe44xxInit (void)
{
    Disable_AFE44x0_SPI_Read();
    AFE44x0_Reg_Write(CONTROL0,0x000000);
    AFE44x0_Reg_Write(CONTROL0,0x000008);
    AFE44x0_Reg_Write(TIAGAIN,0x000000); // CF = 5pF, RF = 500kR
    AFE44x0_Reg_Write(TIA_AMB_GAIN,0x000001);

    AFE44x0_Reg_Write(LEDCNTRL,0x001414);
    AFE44x0_Reg_Write(CONTROL2,0x000000); // LED_RANGE=100mA, LED=50mA
    AFE44x0_Reg_Write(CONTROL1,0x010707); // Timers ON, average 3 samples
    AFE44x0_Reg_Write(PRPCOUNT, 0X001F3F); // 7999
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

    __delay_cycles(1000);
    Enable_AFE44x0_SPI_Read();
}

/*********************************************************************
 * AFE44xx_PowerOn_Init() - Initialize GPIO and SPI communication
 *********************************************************************/
void AFE44xx_PowerOn_Init(void)
{
  volatile unsigned short Init_i, j;
  Init_AFE44xx_Resource(); // Set GPIO and SPI Pins
  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);
  }
  Init_AFE44xx_DRDY_Interrupt(); // Configure Data Ready Interrupt
  afe44xxInit(); // Initialize AFE with its Register values
}

/*************************************************************
 * AFE44x0_Reg_Write() - AFE4490 SPI Register Write
 *************************************************************/
void AFE44x0_Reg_Write (unsigned char reg_address, unsigned long data)
{
  unsigned char dummy_rx;

  P4OUT&= ~0x01;   //  SEN LOW FOR TRANSMISSION.
  // Loop unrolling for machine cycle optimization
  UCB1TXBUF = reg_address;                    // Send the first byte to the TX Buffer: Address of register
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = (unsigned char)(data >>16);     // Send the second byte to the TX Buffer: Data[23:16]
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = (unsigned char)(((data & 0x00FFFF) >>8));       // Send the third byte to the TX Buffer: Data[15:8]
  while ( (UCB1STAT & UCBUSY) );                        // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;                         // Dummy Read Rx buf
  UCB1TXBUF = (unsigned char)(((data & 0x0000FF)));           // Send the first byte to the TX Buffer: Data[7:0]
  while ( (UCB1STAT & UCBUSY) );                        // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;                         // Dummy Read Rx buf
  P4OUT|=0x01;  // SEN HIGH
}

/**********************************************************************************
 * AFE44x0_Reg_Read() - AFE SPI Read Registet
 **********************************************************************************/
unsigned long AFE44x0_Reg_Read(unsigned char Reg_address)
{
  unsigned char SPI_Rx_buf[4];
  unsigned long retVal;
  retVal = 0;
  P4OUT&= ~0x01;   //  SEN LOW FOR TRANSMISSION.
  // Loop unrolling for machine cycle optimization
  UCB1TXBUF = Reg_address;                    // Send the first byte to the TX Buffer: Address of register
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  SPI_Rx_buf[0] = UCB1RXBUF;            // Read Rx buf
  UCB1TXBUF = 0;                              // Send the second byte to the TX Buffer: dummy data
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  SPI_Rx_buf[1] = UCB1RXBUF;            // Read Rx buf: Data[23:16]
  UCB1TXBUF = 0;                              // Send the third byte to the TX Buffer: dummy data
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  SPI_Rx_buf[2] = UCB1RXBUF;            // Read Rx buf: Data[15:8]
  UCB1TXBUF = 0;                              // Send the first byte to the TX Buffer: dummy data
  while ( (UCB1STAT & UCBUSY) );        // USCI_B1 TX buffer ready?
  SPI_Rx_buf[3] = UCB1RXBUF;            // Read Rx buf: Data[7:0]
  P4OUT|=0x01;  // set HIGH at end of transmission
  retVal = SPI_Rx_buf[1];
  retVal = (retVal << 8) | SPI_Rx_buf[2];
  retVal = (retVal << 8) | SPI_Rx_buf[3];

  return    retVal;
}

/***********************************************************************
 * Enable_AFE44x0_SPI_Read() - Enable SPI_Read
 ***********************************************************************/
void Enable_AFE44x0_SPI_Read (void)
{
  unsigned char dummy_rx;
  //Set Control0 - Enable SPI Read bit
  P4OUT&= ~0x01;                        // CS LOW for start of transmission.
  // Loop unrolling for machine cycle optimization
  UCB1TXBUF = 0;                        // Send the first byte to the TX Buffer: Address of register
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 0;                        // Send the second byte to the TX Buffer: Data[23:16]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 0;                        // Send the third byte to the TX Buffer: Data[15:8]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 1;                        // Send the first byte to the TX Buffer: Data[7:0]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  P4OUT|=0x01;                          // CS High for end of transmission
}

/***********************************************************************
 * Disable_AFE44x0_SPI_Read() - Disable SPI_Read
 ***********************************************************************/
void Disable_AFE44x0_SPI_Read (void)
{
  unsigned char dummy_rx;
  //Set Control0 - Disable SPI Read bit
  P4OUT&= ~0x01;                        // CS LOW for start of transmission.
  // Loop unrolling for machine cycle optimization
  UCB1TXBUF = 0;                        // Send the first byte to the TX Buffer: Address of register
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 0;                        // Send the second byte to the TX Buffer: Data[23:16]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 0;                        // Send the third byte to the TX Buffer: Data[15:8]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  UCB1TXBUF = 0;                        // Send the first byte to the TX Buffer: Data[7:0]
  while ( (UCB1STAT & UCBUSY) );    // USCI_B1 TX buffer ready?
  dummy_rx = UCB1RXBUF;         // Dummy Read Rx buf
  P4OUT|=0x01;                          // CS High for end of transmission
}

/*************************************************************************
 * SPI Interrupt Service Routine
 *************************************************************************/
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
  switch(UCB1IV)
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:break;                             // Vector 2 - RXIFG
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

/*****************************************************************************************
 * End of file
 *****************************************************************************************/

