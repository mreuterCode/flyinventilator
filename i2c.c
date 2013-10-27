#include <msp430.h>

/*
 *  * Sensortest
 *  * http://flyinventilator.wordpress.com - maximilian reuter
 */

unsigned char TransmittedData;
unsigned char ReceivedData;
int WriteMode = 0;
int phase = 0;
unsigned char WriteRegister;
int result;
int result2;

void wait(int time);
void i2cSetup(int Adress);
void i2cWrite(int Adress, int Value);
unsigned char i2cRequest(unsigned char Register);
void i2cSend(unsigned char Data);

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;         // Stop WDT

  i2cSetup(0x69);			//setup the I2C connection
  result = i2cRequest(0x6B);		//look if sensor in sleep mode (result = 0x40)
  i2cWrite(0x6B, 0);			//wake device up: write 0x00 into 0x6B
  result2 = i2cRequest(0x6B);		//check if device woke up
  result = i2cRequest(0x3B);		//read ACCEL_XOUT_H
  result2 = i2cRequest(0x3C);		//read ACCEL_XOUT_L

  for(;;){}				//loop

}

void i2cWrite(int Adress, int Value){
    while (UCB0CTL1 & UCTXSTP);			//check if STOP is sent
	WriteMode = 1;				//important for the ISR procedure
	WriteRegister = Adress;
	TransmittedData = Value;
  	phase = 0;				//important for ISR procedure

	IE2 &= ~UCB0RXIE;			//clear RX interrupt-enable...
	IE2 |= UCB0TXIE;			//... and set TX interrupt-enable

    UCB0CTL1 |= UCTR + UCTXSTT;			//send START condition + address + Write
    __bis_SR_register(CPUOFF + GIE);		//go into a LPM and wait for ISR

	WriteMode = 0;
	return;
}

unsigned char i2cRequest(unsigned char Register){
		TransmittedData = Register;
		WriteMode = 0;				//important for ISR procedure
	  	phase = 0;				//important for ISR procedure
		IE2 &= ~UCB0RXIE;			//clear RX interrupt-enable
		IE2 |= UCB0TXIE;			//set TX interrupt-enable

	    while (UCB0CTL1 & UCTXSTP);			//ensure STOP condition was sent
	    UCB0CTL1 |= UCTR + UCTXSTT;			//send START condition + address + Write
	    __bis_SR_register(CPUOFF + GIE);		//go into a LPM and wait for ISR

	    while (UCB0CTL1 & UCTXSTP);			//ensure STOP was sent
	   	return ReceivedData;
}

void i2cSetup(int Adress){
	  P1SEL |= BIT6 + BIT7;				//declare P1.6 and P1.7 as I2c pins
	  P1SEL2|= BIT6 + BIT7;
	  UCB0CTL1 |= UCSWRST;				//stop UCB0
	  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		//configure UCB0
	  UCB0CTL1 = UCSSEL_2 + UCSWRST;
	  UCB0BR0 = 12;
	  UCB0BR1 = 0;
	  UCB0I2CSA = Adress;				//set sensor's I2C address
	  UCB0CTL1 &= ~UCSWRST;				//start UCB0
	  IE2 |= UCB0TXIE + UCB0RXIE;			//enable TX and RX interrupts
	  return;
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	if(WriteMode){
		if(phase == 0){
			UCB0TXBUF = WriteRegister;		//put the register address on the bus
			phase = 1;

		} else if (phase == 1){
			phase = 2;
			UCB0TXBUF = TransmittedData;		//put data on the bus

		}else if(phase == 2)
			IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);	//disable RX and TX interrupts
			UCB0CTL1 |= UCTXSTP;			//send STOP condition
			__bic_SR_register_on_exit(CPUOFF);	//exit LPM
		    WriteMode = 0;
		    phase = 0;
		}
	}else{
		//Read-mode
		if(phase == 0){
			UCB0TXBUF = TransmittedData;		//put register address on the bus
			phase = 1;

		}else if (phase == 1)
		{
			IE2 &= ~UCB0TXIE;			//disable TX interrupts
			IE2 |= UCB0RXIE;			//enable RX interrupts

			UCB0CTL1 &= ~UCTR;			//clear UCTR bit (means READ mode on next START condition)
		    UCB0CTL1 |= UCTXSTT;			//send START condition

		    while (UCB0CTL1 & UCTXSTT);			//wait for START to be sent
		    UCB0CTL1 |= UCTXSTP;			//send STOP condition and wait for the sensor-data
			phase = 2;

		}
		else if (phase == 2)
		{
			ReceivedData = UCB0RXBUF;		//read sensor-data from the bus
			IFG2 &= ~(UCB0TXIFG + UCB0RXIFG)	//disable interrupts
			__bic_SR_register_on_exit(CPUOFF);	//exit LPM

		}

	}
}
