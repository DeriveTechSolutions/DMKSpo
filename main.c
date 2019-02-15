#include "driverlib.h"
#include <msp430.h>
#include "SpO2.h"

int main(void)
{
    WDT_A_hold(WDT_A_BASE);
    SpO2_init();
    Start_SpO2();
    //Stop_SpO2();
}
