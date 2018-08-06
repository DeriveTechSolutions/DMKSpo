/*
 *  Port 1.1 - AFE_RESETZ, P1.2 - AFE_PDNZ, P2.3 - ADC_RDY, P2.4- PD_ALM, P2.5 - LED_ALM, P5.7 - DIAG_END
 *
 */

#include <msp430.h> 
#include <string.h>
#include "uart.h"
#include "main.h"
#include "types.h"
#include "device.h"
#include "AFE44x0.h"

/**
 * main.c
 */

void Init_Ports (void);
BYTE retInString (char* string);
unsigned char ascii2uint8 (unsigned char asciiVal);
unsigned char asciiValArr[4];

//Global flags set by events
volatile BYTE bCDCDataReceived_event = FALSE;   // Indicates data has been received without an open rcv operation

#define MAX_STR_LENGTH 64
char wholeString[MAX_STR_LENGTH] = "";          // The entire input string from the last 'return'
char outString[MAX_STR_LENGTH] = "";            // Holds the outgoing string

unsigned long AFE44xx_SPO2_Data_buf[6];
unsigned char txString[MAX_STR_LENGTH] = "";
//char startCaptureFlag = 0;

enum CAP_MODE {FINITE, CONTINUOUS};
char captureMode = FINITE;
char captureInProgressFlag = 0;

char sendDataFlag = 0;
char readDataFlag = 0;

unsigned long AFE44xxRegArr[49];
unsigned char AFE44xxRegAddr[49];

unsigned char AFE44xxAddr;
unsigned long AFE44xxRegVal;
unsigned long totalCount;
unsigned long sampleCount;

unsigned char CALIBRATION_ENABLED = 0;

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	//initializing
	Init_Ports ();
	uart_init(9600);
	AFE44xx_PowerOn_Init();
	AFE44xx_Read_All_Regs( AFE44xxRegArr);

	__enable_interrupt();                           //Enable interrupts globally

	while(1)
	{
	    if(readDataFlag)
	    {
	        readDataFlag = 0;
	          AFE44xx_SPO2_Data_buf[0] = AFE44x0_Reg_Read(42);  //read RED Data
	          AFE44xx_SPO2_Data_buf[1] = AFE44x0_Reg_Read(43);  //read Ambient data
	          AFE44xx_SPO2_Data_buf[2] = AFE44x0_Reg_Read(44);  //read IR Data
	          AFE44xx_SPO2_Data_buf[3] = AFE44x0_Reg_Read(45);  //read Ambient Data
	          AFE44xx_SPO2_Data_buf[4] = AFE44x0_Reg_Read(46);  //read RED - Ambient Data
	          AFE44xx_SPO2_Data_buf[5] = AFE44x0_Reg_Read(47);  //read IR - Ambient Data
	    }

	    if (sendDataFlag)
	    {
	      sendDataFlag = 0;
	          txString[0] = (unsigned char) START_READ_ADC_REG_CMD;
	          txString[1] = (unsigned char) SOT;

	          txString[2] = (unsigned char)(AFE44xx_SPO2_Data_buf[0] & 0x000000FF);
	          txString[3] = (unsigned char)((AFE44xx_SPO2_Data_buf[0] & 0x0000FF00) >> 8);
	          txString[4] = (unsigned char)((AFE44xx_SPO2_Data_buf[0] & 0x00FF0000) >> 16);
	          txString[5] = (unsigned char)(AFE44xx_SPO2_Data_buf[1] & 0x000000FF);
	          txString[6] = (unsigned char)((AFE44xx_SPO2_Data_buf[1] & 0x0000FF00) >> 8);
	          txString[7] = (unsigned char)((AFE44xx_SPO2_Data_buf[1] & 0x00FF0000) >> 16);
	          txString[8] = (unsigned char)(AFE44xx_SPO2_Data_buf[2] & 0x000000FF);
	          txString[9] = (unsigned char)((AFE44xx_SPO2_Data_buf[2] & 0x0000FF00) >> 8);
	          txString[10] = (unsigned char)((AFE44xx_SPO2_Data_buf[2] & 0x00FF0000) >> 16);
	          txString[11] = (unsigned char)(AFE44xx_SPO2_Data_buf[3] & 0x000000FF);
	          txString[12] = (unsigned char)((AFE44xx_SPO2_Data_buf[3] & 0x0000FF00) >> 8);
	          txString[13] = (unsigned char)((AFE44xx_SPO2_Data_buf[3] & 0x00FF0000) >> 16);
	          txString[14] = (unsigned char)(AFE44xx_SPO2_Data_buf[4] & 0x000000FF);
	          txString[15] = (unsigned char)((AFE44xx_SPO2_Data_buf[4] & 0x0000FF00) >> 8);
	          txString[16] = (unsigned char)((AFE44xx_SPO2_Data_buf[4] & 0x00FF0000) >> 16);
	          txString[17] = (unsigned char)(AFE44xx_SPO2_Data_buf[5] & 0x000000FF);
	          txString[18] = (unsigned char)((AFE44xx_SPO2_Data_buf[5] & 0x0000FF00) >> 8);
	          txString[19] = (unsigned char)((AFE44xx_SPO2_Data_buf[5] & 0x00FF0000) >> 16);

	          txString[20] = (unsigned char) EOT;
	          txString[21] = (unsigned char) CR;
	    }

	    if (retInString(wholeString))                                  //Has the user pressed return yet?
	    {
	          if (wholeString[0] == WRITE_REG_CMD) // AFE44xx Write Operation
	          {
	              if (CALIBRATION_ENABLED == 0)
	              {
	                  AFE44xxAddr = (ascii2uint8 (wholeString[1]) << 4) | ascii2uint8 (wholeString[2]);
	                  unsigned long AFE44xxRegData[3];
	                  AFE44xxRegData[0] = (ascii2uint8 (wholeString[3]) << 4) | ascii2uint8 (wholeString[4]);
	                  AFE44xxRegData[1] = (ascii2uint8 (wholeString[5]) << 4) | ascii2uint8 (wholeString[6]);
	                  AFE44xxRegData[2] = (ascii2uint8 (wholeString[7]) << 4) | ascii2uint8 (wholeString[8]);
	                  AFE44xxRegVal = (AFE44xxRegData[0]<<16)| (AFE44xxRegData[1]<<8) | (AFE44xxRegData[2]);

	                      // Reg_Addr != 0; Disable READ; Write value to REG; Enable READ
	                      if (AFE44xxAddr)
	                      {
	                          Disable_AFE44x0_SPI_Read();
	                          AFE44x0_Reg_Write(AFE44xxAddr, AFE44xxRegVal);
	                          Enable_AFE44x0_SPI_Read();
	                      }
	                      // Reg_Addr == 0; Write value to REG;
	                      else
	                          AFE44x0_Reg_Write(AFE44xxAddr, AFE44xxRegVal);
	              }

	          }
	          else if (wholeString[0] == READ_REG_CMD) // AFE44xx Read Operation
	          {
	              AFE44xxAddr = (ascii2uint8 (wholeString[1]) << 4) | ascii2uint8 (wholeString[2]);
	              // Reg_Addr != 0; Read REG;
	              if (AFE44xxAddr)
	              {
	                  Enable_AFE44x0_SPI_Read();
	                  AFE44xxRegVal = AFE44x0_Reg_Read(AFE44xxAddr);
	              }
	              else
	                  AFE44xxRegVal = 0;
	                  txString[0] = (unsigned char) READ_REG_CMD;
	                  txString[1] = (unsigned char) SOT;
	                  txString[2] = (unsigned char)(AFE44xxRegVal & 0x000000FF);
	                  txString[3] = (unsigned char)((AFE44xxRegVal & 0x0000FF00) >> 8);
	                  txString[4] = (unsigned char)((AFE44xxRegVal & 0x00FF0000) >> 16);
	                  txString[5] = (unsigned char) EOT;
	                  txString[6] = (unsigned char) CR;
	          }
	          else if (wholeString[0] == START_READ_ADC_REG_CMD) // Start AFE44x0 ADC Reg Read Command
	          {
	                     if (captureInProgressFlag == 0)
	                     {
	                         captureInProgressFlag = 1;
	                         totalCount = 1;

	                         wholeString[2] = (ascii2uint8 (wholeString[2]) << 4) | ascii2uint8 (wholeString[3]);
	                         wholeString[3] = (ascii2uint8 (wholeString[4]) << 4) | ascii2uint8 (wholeString[5]);
	                         wholeString[4] = (ascii2uint8 (wholeString[6]) << 4) | ascii2uint8 (wholeString[7]);
	                         wholeString[5] = (ascii2uint8 (wholeString[8]) << 4) | ascii2uint8 (wholeString[9]);

	                         sampleCount = (unsigned long)wholeString[5];
	                         sampleCount += (unsigned long)wholeString[4] * 256;
	                         sampleCount += (unsigned long)wholeString[3] * 65536;
	                         sampleCount += (unsigned long)wholeString[2] * 16777216;

	                         if (sampleCount)
	                         {
	                             captureMode = FINITE;
	                             totalCount = sampleCount;
	                         }
	                         else
	                             captureMode = CONTINUOUS;

	                         sampleCount = 0;
	                         Enable_AFE44xx_DRDY_Interrupt();          // Enable DRDY interrupt

	                         readDataFlag = 0;
	                         sendDataFlag = 0;
	                         AFE44xx_SPO2_Data_buf[0] = 0;
	                         AFE44xx_SPO2_Data_buf[1] = 0;
	                         AFE44xx_SPO2_Data_buf[2] = 0;
	                         AFE44xx_SPO2_Data_buf[3] = 0;
	                         AFE44xx_SPO2_Data_buf[4] = 0;
	                         AFE44xx_SPO2_Data_buf[5] = 0;
	                     }
	          }

	          else if (wholeString[0] == STOP_READ_ADC_REG_CMD) // Stop AFE44x0 ADC Reg Read Command
	          {
	                     Disable_AFE44xx_DRDY_Interrupt();
	                     sampleCount = 0;
	                     totalCount = 1;
	                     readDataFlag = 0;
	                     sendDataFlag = 0;
	                     captureMode = FINITE;
	                     captureInProgressFlag = 0;
	                     //P5OUT &= ~BIT0;
	          }//Turn off LED P5.0 (Green)
	    }
	}
}

void Init_Ports (void)
{
  //Initialization of ports (all unused pins as outputs with low-level)
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

unsigned char ascii2uint8 (unsigned char asciiVal)
{
  unsigned char uint8Val;

  if (asciiVal > 0x60 && asciiVal < 0x67)           // 'a' to 'f'
    uint8Val = ((asciiVal - 0x61) + 0x0A);
  else if (asciiVal > 0x40 && asciiVal < 0x47)      // 'A' to 'F'
    uint8Val = ((asciiVal - 0x41) + 0x0A);
  else if (asciiVal > 0x2F && asciiVal < 0x3A)      // '0' to '9'
    uint8Val = (asciiVal - 0x30);
  else                                              // others
    uint8Val = 0;

  return uint8Val;
}

BYTE retInString (char* string)
{
  BYTE retPos = 0,i,len;
  char tempStr[MAX_STR_LENGTH] = "";

  strncpy(tempStr,string,strlen(string));                     //Make a copy of the string
  len = strlen(tempStr);
  while ((tempStr[retPos] != 0x0A) && (tempStr[retPos] != 0x0D) &&
         (retPos++ < len)) ;                                  //Find 0x0D; if not found, retPos ends up at len

  if ((retPos < len) && (tempStr[retPos] == 0x0D)){           //If 0x0D was actually found...
    for (i = 0; i < MAX_STR_LENGTH; i++){                   //Empty the buffer
      string[i] = 0x00;
    }
    strncpy(string,tempStr,retPos);                         //...trim the input string to just before 0x0D
    return ( TRUE) ;                                        //...and tell the calling function that we did so
  } else if ((retPos < len) && (tempStr[retPos] == 0x0A)){    //If 0x0D was actually found...
    for (i = 0; i < MAX_STR_LENGTH; i++){                   //Empty the buffer
      string[i] = 0x00;
    }
    strncpy(string,tempStr,retPos);                         //...trim the input string to just before 0x0D
    return ( TRUE) ;                                        //...and tell the calling function that we did so
  } else if (tempStr[retPos] == 0x0D){
    for (i = 0; i < MAX_STR_LENGTH; i++){                   //Empty the buffer
      string[i] = 0x00;
    }
    strncpy(string,tempStr,retPos);                         //...trim the input string to just before 0x0D
    return ( TRUE) ;                                        //...and tell the calling function that we did so
  } else if (retPos < len){
    for (i = 0; i < MAX_STR_LENGTH; i++){                   //Empty the buffer
      string[i] = 0x00;
    }
    strncpy(string,tempStr,retPos);                         //...trim the input string to just before 0x0D
    return ( TRUE) ;                                        //...and tell the calling function that we did so
  }

  return ( FALSE) ;                                           //Otherwise, it wasn't found
}
