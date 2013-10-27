#include <msp430.h> 

/*
 * 	* GPIO Test
 *  * http://flyinventilator.wordpress.com - maximilian reuter
 *	* All code available under https://github.com/mreuterCode/flyinventilator
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    P2SEL &= ~(BIT6 + BIT7);  //P2.6 and P2.7 have set a P2SEL bit initially for clock purpose. Clear before use!

    //set all ports high
    P1OUT |= BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5+ BIT6 + BIT7;
    P2OUT |= BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5+ BIT6 + BIT7;
    P3OUT |= BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5+ BIT6 + BIT7;

    //declare all ports output
    P1DIR = BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5+ BIT6 + BIT7;
    P2DIR = BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5 + BIT6 + BIT7;
    P3DIR = BIT0+BIT1+BIT2+BIT3 +BIT4+BIT5+ BIT6 + BIT7;

    while(1);
	return 0;
}
