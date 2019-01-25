#include <msp430.h>
#include "stdint.h"
#include"device.h"
#include "UART.h"
#include"AFE44x0.h"
#include "HAL_UCS.h"
#include "HAL_PMM.h"

/*******************************************************************
 * Macros
 *******************************************************************/
#define FS            25    //sampling frequency
#define BUFFER_SIZE  (FS*4)
#define MA4_SIZE  4 // DONOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))
#define false   0
#define true    1

/********************************************************************
 * Global Variables
 ********************************************************************/
volatile int drdy_trigger = 0;
volatile int afe44xx_data_ready = false;
unsigned long IRtemp,REDtemp;
static  int32_t an_x[ BUFFER_SIZE];
static  int32_t an_y[ BUFFER_SIZE];
volatile int8_t n_buffer_count;
int data=0;
int dec=0;
signed long seegtemp=0, seegtemp2=0;

int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data

/*******************************************************************************************
 * Lookup Table for spo2
 *******************************************************************************************/
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
              3, 2, 1 } ;

/***************************************************************************************
 * Function Prototypes
 ***************************************************************************************/

void Init_Ports (void);
void Init_Clock (void);
void Init_TimerA1 (void);
void clock_config_16(void);
void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num );
void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height );
void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void sort_ascend(int32_t  *pn_x, int32_t n_size);
void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);
int afe_data;

/***********************************************************************************
 * main function
 ***********************************************************************************/
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog timer

    P4OUT = BIT7;
    P4DIR = BIT7;
    P4OUT = BIT7;
    Init_Ports();  // Initialize ports (first ports because clocks do change ports)
    SetVCore(3);
    clock_config_16(); // Configure Clock to 16MHz
    AFE44xx_PowerOn_Init(); // Configure GPIO, UCB1 as SPI, and Data Ready Interrupt
    uart_config(); // Configure UART settings at Baud Rate as 115200

    __enable_interrupt(); // Enable Interrupt
    __delay_cycles(1000);

    while(1)
    {
            P4OUT ^= BIT7; // Toggle Green LED
            if (drdy_trigger == 1) // Sets drdy_trigger if ISR of Port_2 is served
            {
                    Disable_AFE44xx_DRDY_Interrupt();
                    Enable_AFE44x0_SPI_Read(); // Enable Reading by setting SPI_Read in CONTROL0
                    IRtemp= AFE44x0_Reg_Read(44); // Read IR data from LED1VAL(address: 0x2A)
                    Enable_AFE44x0_SPI_Read();
                    REDtemp = AFE44x0_Reg_Read(42); // Read Red data from LED2VAL(address: 0x2C)
                    afe44xx_data_ready = true;
                    //send_data(IRtemp);
                    //send_string("\r\n");
                    if(IRtemp == 2096921) // To check if Finger is inserted into the probe
                    {
                        send_string("Insert Finger!!!");
                        send_string("\r\n");
                    }
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

                            if(n_spo2 == -999) // Invalid data
                            {
                              send_string("Probe error!!!!");
                              send_string("\r\n");
                            }
                            else
                            {
                              send_string("\r\n");
                              send_string("calculating sp02...");
                              send_string("\r\n");
                              send_string(" Sp02 : ");
                              send_data(n_spo2);

                              send_string("%");
                              send_string("\r\n");
                              send_string("Pulse rate :");
                              send_data(n_heart_rate);
                              send_string("\r\n");
                            }
                            n_buffer_count = 0;
                    }
                    afe44xx_data_ready = false;
                    drdy_trigger = 0;
            }
            Enable_AFE44xx_DRDY_Interrupt();
    }
}

/*******************************************************************
 * Data Ready Interrupt Service Routine
 *******************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    switch (P2IV)
    {
        case P2IV_P2IFG6:
            P2IFG &= ~BIT6;                                   // Clear P2.3 IFG i.e Data RDY interrupt status
            data = 1;
            drdy_trigger = 1;                                 // Set Flag to read AFE44x0 ADC REG data
            break;

        case P2IV_NONE:
            break;
    }
}

/*******************************************************************************
 * Initialization of ports (all unused pins as outputs with low-level)
 *******************************************************************************/
void Init_Ports (void)
{
  P1OUT = 0xFF;
  P1DIR = 0xFF;
  P2OUT = 0xFF;
  P2DIR = 0xFF;
  P3OUT = 0x00;
  P3DIR = 0xFF;
  P4OUT = 0x00;
  P4DIR = 0xFF;
  P5OUT = 0x00;
  P5DIR = 0xFF;
  P6OUT = 0x00;
  P6DIR = 0xFF;
  P7OUT = 0x00;
  P7DIR = 0xFF;
  P8OUT = 0x00;
  P8DIR = 0xFF;
#if defined (__MSP430F563x_F663x)
  P9OUT = 0x00;
  P9DIR = 0xFF;
#endif
}

/**********************************************************************************************
 * Init_Clock
 **********************************************************************************************/
void Init_Clock (void)
{
  //Initialization of clock module
  if (/*USB_PLL_XT == 2*/1)
  {
#if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x0C;  //enable XT2 pins for F5529
#elif defined (__MSP430F563x_F663x)
    P7SEL |= 0x0C;
#endif

    //use REFO for FLL and ACLK
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
    //Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //Start the FLL, at the freq indicated by the config
    //constant USB_MCLK_FREQ
    XT2_Start(XT2DRIVE_2);  //Start the "USB crystal"
  }
  else
  {
#if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x10;   //enable XT1 pins
#endif
    //Use the REFO oscillator to source the FLL and ACLK
    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
    //Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //set FLL (DCOCLK)

    XT1_Start(XT1DRIVE_0);  //Start the "USB crystal"
  }
}

/***********************************************************************************************************
 * CALCULATE SPO2 AND PULSE RATE
 * Using Peak Detector as Valley Detector and 4-Point Moving Average
 * Function Parameters:
 * 1. pun_ir_buffer - IR data
 * 2. n_ir_buffer_length - 100
 * 3. pun_red_buffer - Red data
 * 4. pn_spo2 - spo2 data (%)
 * 5. pch_spo2_valid - Flag indicating true spo2
 * 6. pn_heart_rate - HeartRate data
 * 7. pch_hr_valid - Flag indicating true HeartRate
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
 * find_peak() - Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
 ****************************************************************************************************/
void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
{
    find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height ); // Find all the peaks above minimum height
    remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance ); // Remove the peaks separated by minimum distance
    *n_npks = min( *n_npks, n_max_num );
}

/************************************************************************************************************
 * find_peak_above() - Find all peaks above MIN_HEIGHT
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
 * remove_close_peaks() - Detect peaks that are at least separated by minimum peak distance(eg. 5), in number of channels
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
 * sort_ascend() - Sort array in ascending order (insertion sort algorithm)
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
 * sort_indices_descend() - Sort indices according to descending order (insertion sort algorithm)
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
 * clock_config_16() - Set Clock Frequency to 16MHz
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

