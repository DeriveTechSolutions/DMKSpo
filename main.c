#include <msp430.h>
#include "stdint.h"
#include"device.h"
#include "UART.h"
#include"AFE44x0.h"
#include "HAL_UCS.h"
#include "HAL_PMM.h"

#define FS            25    //sampling frequency
#define BUFFER_SIZE  (FS*4)
#define MA4_SIZE  4 // DONOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))
#define false   0
#define true    1


volatile int drdy_trigger = 0;
volatile int afe44xx_data_ready = false;
unsigned long IRtemp,REDtemp;
static  int32_t an_x[ BUFFER_SIZE];
static  int32_t an_y[ BUFFER_SIZE];
volatile int8_t n_buffer_count;
int data=0;

int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
long status_byte=0;
uint8_t LeadStatus=0;
uint8_t leadoff_deteted = true;
uint8_t spo2_probe_open = false;
int dec=0;
signed long seegtemp=0, seegtemp2=0;
/**
 * main.c
 */
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
void Init_Ports (void);
void Init_Clock (void);
void Init_TimerA1 (void);
void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num );
void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height );
void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void sort_ascend(int32_t  *pn_x, int32_t n_size);
void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);
int afe_data;
void timer0_config();
int main(void)
{


    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    P4OUT=BIT7;
    P4DIR=BIT7;
    P4OUT=BIT7;
Init_Ports();                                               //Init ports (do first ports because clocks do change ports)
      SetVCore(3);
     // uart_config();
    //  Init_Clock();

    clock_config_16();
      //timer0_config();
      AFE44xx_PowerOn_Init();
      uart_config();
//while(1)
//{

  //  P4OUT^=BIT7;
  //  __delay_cycles(1000000);
//}
      __enable_interrupt();
__delay_cycles(1000);
//uart_tx('A');
//Enable_AFE44x0_SPI_Read ();
//afe_data=AFE44x0_Reg_Read(LED2ENDC);
        // Disable_AFE44x0_SPI_Read ();
      while(1)
    {
          P4OUT^=BIT7;
        if (drdy_trigger == 1)
        {
    //        uart_tx('B');
          //  detachInterrupt(0);
            Disable_AFE44xx_DRDY_Interrupt();
            Enable_AFE44x0_SPI_Read();
            IRtemp= AFE44x0_Reg_Read(44);
            Enable_AFE44x0_SPI_Read();
            REDtemp = AFE44x0_Reg_Read(42);
            afe44xx_data_ready = true;
            send_data( IRtemp);
            send_string("\r\n");
        }
        if(afe44xx_data_ready == true)
        {
      //      uart_tx('C');
                IRtemp = (unsigned long) (IRtemp<<10);
                seegtemp = (signed long) (IRtemp);
                seegtemp = (signed long) (seegtemp>>10);

                REDtemp = (unsigned long) (REDtemp<<10);
                seegtemp2 = (signed long) (REDtemp);
                seegtemp2 = (signed long) (seegtemp2>>10);

                if(dec==20)
                     {
                       aun_ir_buffer[n_buffer_count]=(uint16_t) (seegtemp>>4);
                       aun_red_buffer[n_buffer_count]=(uint16_t) (seegtemp2>>4);
                       n_buffer_count++;
                       dec=0;
        //               uart_tx('D');
                     }


                     dec++;

                     if(n_buffer_count>99)
                     {

                       estimate_spo2(aun_ir_buffer, 100, aun_red_buffer, &n_spo2, &ch_spo2_valid,&n_heart_rate, &ch_hr_valid);

                     //  send_data(reverse(n_spo2));
                          if(n_spo2 == -999){
                         //   send_string("Probe error!!!!");
                          //  send_string("\r\n");
                          }
                          else
                          {
//                              send_string("\r\n");
//                              send_string("calculating sp02...");
//                              send_string("\r\n");
//                             send_string(" Sp02 : ");
//                             send_data(n_spo2);
//
//                              send_string("%");
//                              send_string("\r\n");
//                           send_string("Pulse rate :");
//                           send_data(n_heart_rate);
//                           send_string("\r\n");
                         //    send_string(" bpm");
                     //  send_string("   ");
                          }
                          n_buffer_count=0;
                        }
                     afe44xx_data_ready = false;
                         drdy_trigger = 0;
                      //   Enable_AFE44xx_DRDY_Interrupt();
                        // attachInterrupt(0, afe44xx_drdy_event, RISING );
        }
        Enable_AFE44xx_DRDY_Interrupt();
    }
    return 0;
}

#pragma vector=PORT2_VECTOR  //DRDY interrupt
__interrupt void Port_2(void)
{
  switch (P2IV)
  {
  case P2IV_P2IFG6:
    P2IFG &= ~BIT6;                                   // Clear P2.3 IFG i.e Data RDY interrupt status
    //P5OUT |= BIT0;
    //test=100;//Turn on LED P5.0 (Green)
    data=1;
    drdy_trigger = 1;                                 // Set Flag to read AFE44x0 ADC REG data
    break;

  case P2IV_NONE:
    break;
  }
 // data=1;
}

void Init_Ports (void)
{
  //Initialization of ports (all unused pins as outputs with low-level
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


void Init_Clock (void)
{
  //Initialization of clock module
  if (/*USB_PLL_XT == 2*/1){
#if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x0C;                                      //enable XT2 pins for F5529
#elif defined (__MSP430F563x_F663x)
    P7SEL |= 0x0C;
#endif

    //use REFO for FLL and ACLK
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
   // Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //Start the FLL, at the freq indicated by the config
    //constant USB_MCLK_FREQ
    XT2_Start(XT2DRIVE_2);                                          //Start the "USB crystal"
  }
  else {
#if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x10;                                      //enable XT1 pins
#endif
    //Use the REFO oscillator to source the FLL and ACLK
    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
  //  Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //set FLL (DCOCLK)

    XT1_Start(XT1DRIVE_0);                                          //Start the "USB crystal"
  }
}


void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  uint32_t un_ir_mean,un_only_once ;
  int32_t k, n_i_ratio_count;
  int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks, n_c_min;
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;

  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx, n_x_dc_max_idx;
  int32_t an_ratio[5], n_ratio_average;
  int32_t n_nume, n_denom ;
 // send_data(n_ir_buffer_length);

  // calculates DC mean and subtract DC from ir
  un_ir_mean =0;
 // int count=0;
  for (k=0 ; k<n_ir_buffer_length ; k++ )
  {
      un_ir_mean += pun_ir_buffer[k] ;
   //  send_data(k);
   //  send_data(n_ir_buffer_length);
    // count++;
  }
  un_ir_mean =un_ir_mean/n_ir_buffer_length ;

  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k=0 ; k<n_ir_buffer_length ; k++ )
    an_x[k] = -1*(pun_ir_buffer[k] - un_ir_mean) ;

  // 4 pt Moving Average
  for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
    an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int)4;
  }

  // calculate threshold
  n_th1=0;
  for ( k=0 ; k<BUFFER_SIZE ;k++){

    n_th1 +=  an_x[k];
  }

  n_th1=  n_th1/ ( BUFFER_SIZE);
  if( n_th1<30) n_th1=30; // min allowed
  if( n_th1>60) n_th1=60; // max allowed

  for ( k=0 ; k<15;k++) an_ir_valley_locs[k]=0;
  // since we flipped signal, we use peak detector as valley detector
  find_peak( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks
  n_peak_interval_sum =0;
  if (n_npks>=2){
    for (k=1; k<n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] -an_ir_valley_locs[k -1] ) ;
    n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
    *pn_heart_rate =(int32_t)( (FS*60)/ n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else  {
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for (k=0 ; k<n_ir_buffer_length ; k++ )  {
      an_x[k] =  pun_ir_buffer[k] ;
      an_y[k] =  pun_red_buffer[k] ;
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count =n_npks;

  //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average =0;
  n_i_ratio_count = 0;
  for(k=0; k< 5; k++) an_ratio[k]=0;
  for (k=0; k< n_exact_ir_valley_locs_count; k++){
    if (an_ir_valley_locs[k] > BUFFER_SIZE ){
      *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0;

      return;
    }
  }
  // find max between two valley locations
  // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
  for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
    n_y_dc_max= -16777216 ;
    n_x_dc_max= -16777216;
    if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3){
        for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++){
          if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i]; n_x_dc_max_idx=i;}
          if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i]; n_y_dc_max_idx=i;}
      }
      n_y_ac= (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
      n_y_ac=  an_y[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ;
      n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw
      n_x_ac= (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
      n_x_ac=  an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
      n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw
      n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
      n_denom= ( n_x_ac *n_y_dc_max)>>7;
      if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
      {
        an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        n_i_ratio_count++;
      }
    }
  }
 // send_string("Red_ac:");
//  send_data(n_y_ac);
//  send_string("\r\n");
//  send_string("Ir_dc_max:");
//  send_data(n_x_dc_max);

//  send_string("\n\r");
//  send_string("Ir_ac:");
//  send_data(n_x_ac);
//  send_string("\r\n");
//  send_string("Red_dc_max:");
//  send_data(n_y_dc_max);
//  send_string("\n\r");
  // choose median value since PPG signal may varies from beat to beat
  sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx= n_i_ratio_count/2;

  if (n_middle_idx >1)
    n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx ];

  if( n_ratio_average>2 && n_ratio_average <184){
    n_spo2_calc= uch_spo2_table[n_ratio_average] ;
    *pn_spo2 = n_spo2_calc ;
    *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else{
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }
}


void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/




{
    find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height );
    remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
    *n_npks = min( *n_npks, n_max_num );
}



void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size-1){
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){
        // find left edge of potential peaks
      n_width = 1;
      while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 ){      // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width+1;
      }
      else
        i += n_width;
    }
    else
      i++;
    //  Serial.println("beat");
  }



}




void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

  int32_t i, j, n_old_npks, n_dist;

  /* Order peaks from large to small */
  sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ ){
    n_old_npks = *pn_npks;
    *pn_npks = i+1;
    for ( j = i+1; j < n_old_npks; j++ ){
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  sort_ascend( pn_locs, *pn_npks );
}




void sort_ascend(int32_t  *pn_x, int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
        pn_x[j] = pn_x[j-1];
    pn_x[j] = n_temp;
  }
}




void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
      pn_indx[j] = pn_indx[j-1];
    pn_indx[j] = n_temp;
  }
}
void clock_config_16(void)
{
   // P2DIR |= BIT2;                            // SMCLK set out to pins
  //   P2SEL |= BIT2;
    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
     UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
     UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    // UCSCTL0 = (31<<8);
     // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
     do
     {
       UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                               // Clear XT2,XT1,DCO fault flags
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

void clock_config_8()
{


      volatile unsigned int i;


      WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
      P1DIR |= BIT1;                            // P1.1 output

      P1DIR |= BIT0;                            // ACLK set out to pins
      P1SEL |= BIT0;
      P2DIR |= BIT2;                            // SMCLK set out to pins
      P2SEL |= BIT2;
      P7DIR |= BIT7;                            // MCLK set out to pins
      P7SEL |= BIT7;
    P4OUT=0x00;
    P4DIR=BIT7;
    P4OUT=BIT7;
      UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
      UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
      UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx

      // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
      do
      {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                                // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
      }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

      __bis_SR_register(SCG0);                  // Disable the FLL control loop
      UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
      UCSCTL2 |= 121 ;//+FLLD_1;                           // Set DCO Multiplier for 8MHz
                                                // (N + 1) * FLLRef = Fdco
                                                // (249 + 1) * 32768 = 8MHz
      __bic_SR_register(SCG0);                  // Enable the FLL control loop

      // Worst-case settling time for the DCO when the DCO range bits have been
      // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
      // UG for optimization.
      // 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
      __delay_cycles(250000);
      // Worst-case settling time for the DCO when the DCO range bits have been
      // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
      // UG for optimization.
      // 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
  //    __delay_cycles(250000);



}

void timer0_config()
{

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
      P6DIR |= 0x01;                            // P1.0 output
      TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
      TA0CCR0 = 63;
      TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR

      __bis_SR_register(LPM0_bits + GIE);

}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  P6OUT ^= 0x01;                            // Toggle P1.0
}
