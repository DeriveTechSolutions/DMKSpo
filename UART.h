/*
 * UART.h
 *
 *  Created on: 26-Nov-2018
 *      Author: Hrishikesh-derive
 */

#ifndef UART_H_
#define UART_H_
int test=0;
unsigned long int uart_data=0,rushi;
unsigned char rx_data[10],num_digit,ch;
void uart_tx(unsigned char data)
{   UCA0TXBUF = data;
    while (!(UCA0IFG & UCTXIFG));


    UCA0IFG &= (~UCTXIFG);


}

char uart_rx()
{
    while (!(UCA0IFG & UCRXIFG));
    UCA0IFG&=~(UCRXIFG);
return  UCA0RXBUF;
}

void receive_string()
{
   int i=0;

    while((ch=uart_rx()))
    {
    rx_data[i++]=ch;
    if(ch==10)
        break;
    }

    rx_data[--i]='\0';
//return rx_data;
}

void send_string(unsigned char * str)
{
while(*str)
{
    uart_tx(*str);
str++;
}



}
unsigned long  int reverse(unsigned long int data)
{

  unsigned int temp_data=0;
  uart_data=0;
rushi=data;
 if(data==0)
     num_digit++;
  while(data)
  {
      temp_data=data%10;
      uart_data=uart_data*10+temp_data;
num_digit++;




      data=data/10;

  }

return uart_data;
}

void send_data(unsigned long int data)
{
    unsigned long int temp_data=0;
    unsigned char dummy=0;
    temp_data=reverse(data);
    while(temp_data)
    {
        uart_tx((temp_data%10)+48);
        temp_data=temp_data/10;
        num_digit--;

    }
    while(num_digit)
    {

        uart_tx('0');
        num_digit--;
    }
   // uart_tx('\r');
   // uart_tx('\n');

num_digit=0;
}
void UART_strcpy(char * wholeString,char *rx_data)
{
    unsigned int i=0;
    while(rx_data[i])
    {

        wholeString[i]=rx_data[i];
        i++;
    }

    wholeString[i]=rx_data[i];

}
void uart_config()
{
    P3SEL = BIT3+BIT4;

    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
     UCA0CTL1 |= UCSSEL_2;                     // CLK = ACLK
     UCA0BR0 = 138;                           // 32kHz/9600=3.41 (see User's Guide)
     UCA0BR1 = 0x00;                           //
     UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
     UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  //   UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt



}



void send_float_data(double data)
{
unsigned long int integer,i=5;
double fraction;
integer=data;
send_data(integer);
uart_tx('.');
fraction=data-(double)integer;
while(i--)
{
integer=fraction * 10;
send_data(integer);
fraction=(fraction*10)-integer;
}
}
/*void uart_data_send(uint32_t data)
{
reverse(data);
while(uart_data)
{
uart_tx(uart_data%10 + 48);
uart_data=uart_data/10;
//num_digit++;
}

}*/
#endif /* UART_H_ */
