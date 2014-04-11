#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//<-- UART
void wait(int time);
void sendUART(char str[]);
void setupUART();
void setMotorNorth(char throttle);
void setMotorEast(char throttle);
void setMotorSouth(char throttle);
void setMotorWest(char throttle);

int i = 0;
char c[99];
int currentTransmissionByte = 0;
//--> UART

//<-- I2C
unsigned char TransmittedData;
unsigned char ReceivedData;
int WriteMode = 0;
int phase = 0;
unsigned char WriteRegister;
int result;
int result2;
int req;
int req2;

void i2cSetup(int Adress);
void i2cWrite(int Adress, int Value);
unsigned char i2cRequest(unsigned char Register);
void i2cSend(unsigned char Data);
//-->

//<-- PWM Setup
int pwmTimerLimit = 250;	//PWM-frequency: 2kHz
int throttleDown; 			//define "throttle"-positions
int throttleUp;
int throttleNeutral;

void setupPWM();
//-->

//<-- I2C Sample Timer
void setupSampleTimer();
int SensorContainer[8];

int low;
int high;
int value;
//-->

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
  {
    while(1);                               // do not load, trap CPU!!
  }

  setupPWM();

  i2cSetup(0x69);

  i2cRequest(0x02);
  result = i2cRequest(0x6B);		//look if sensor in sleep mode (result = 0x40)
  i2cWrite(0x6B, 0);				//wake device up: write 0x00 into 0x6B
  result2 = i2cRequest(0x6B);		//check if device woke up
  result = i2cRequest(0x3B);		//read ACCEL_XOUT_H
  result2 = i2cRequest(0x3C);		//read ACCEL_XOUT_L
  result = i2cRequest(0x3B);		//read ACCEL_XOUT_H
  result2 = i2cRequest(0x3C);		//read ACCEL_XOUT_L

  setupUART();

  setupSampleTimer();

  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  for(;;);
}

void setupSampleTimer(){
	TACCR0 = 6250;
	//enable interrupts on compare 0
	TACCTL0 = CCIE;
	//set up and start timer
	TACTL = MC_2|ID_3|TASSEL_2|TACLR;
	__enable_interrupt ();
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR (void){
	TACCTL0 &= ~CCIE;	//to be sure not to be interrupted, clear TA0 interrupt

	//ACCX ~0.011s per block
	low = i2cRequest(0x3C);
	high = i2cRequest(0x3B);
	value = high << 8;
	SensorContainer[0] = value + low;

	//ACCY
	low = i2cRequest(0x3E);
	high = i2cRequest(0x3D);
	value = high << 8;
	SensorContainer[1] = value + low;

	//ACCZ
	low = i2cRequest(0x40);
	high = i2cRequest(0x3F);
	value = high << 8;
	SensorContainer[2] = value + low;

	//GYRX
	low = i2cRequest(0x44);
	high = i2cRequest(0x43);
	value = high << 8;
	SensorContainer[3] = value + low;

	//GYRY
	low = i2cRequest(0x46);
	high = i2cRequest(0x45);
	value = high << 8;
	SensorContainer[4] = value + low;

	//GYRZ
	low = i2cRequest(0x48);
	high = i2cRequest(0x47);
	value = high << 8;
	SensorContainer[5] = value + low;

	//Temp
	low = i2cRequest(0x42);
	high = i2cRequest(0x41);
	value = high << 8;
	SensorContainer[6] = value + low;

	char buffer[50];
	sprintf(buffer, "%d, %d, %d, %d, %d, %d, %d \n", SensorContainer[0], SensorContainer[1], SensorContainer[2], SensorContainer[3], SensorContainer[4], SensorContainer[5], SensorContainer[6]);
	sendUART(buffer);		//send the whole CSV string

	TACCTL0 = CCIE;			//reactivate interrupts!

}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	if(UCA0RXBUF != 0){
		c[i] = UCA0RXBUF;					//write the read value into c
		i++;
		if(i == 98){
			i = 0;
		}

		//apply the motor speed according to the protocol: "BGN" + MOTORN + MOTORE + MOTORS + MOTORW
		if(currentTransmissionByte == 0){
			if(UCA0RXBUF == 'B'){
				currentTransmissionByte++;
			}

		} else if (currentTransmissionByte == 1){
			if(UCA0RXBUF == 'G'){
				currentTransmissionByte++;
			} else{
				currentTransmissionByte = 0;
			}

		} else if (currentTransmissionByte == 2){
			if(UCA0RXBUF == 'N'){
				currentTransmissionByte++;
			} else{
				currentTransmissionByte = 0;
			}

		} else if (currentTransmissionByte == 3){
			setMotorNorth(UCA0RXBUF);
			currentTransmissionByte++;

		} else if (currentTransmissionByte == 4){
			setMotorEast(UCA0RXBUF);
			currentTransmissionByte++;

		} else if (currentTransmissionByte == 5){
			setMotorSouth(UCA0RXBUF);
			currentTransmissionByte++;

		} else if (currentTransmissionByte == 6){
			setMotorWest(UCA0RXBUF);
			currentTransmissionByte = 0;

		}
	}

}

void setupUART(){
	  P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 |= BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	  UCA0BR0 = 52;                             // 1MHz 19200
	  UCA0BR1 = 0;                              // 1MHz 19200
	  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	  return;
}

void sendUART(char str[]){

	//str[strlen(str)] = 0xD;
	int b = 0;
	int d = strlen(str);
	for (b = 0; b< d; b++){
		wait(100);

		while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
		UCA0TXBUF = str[b];
		wait(100);
	}
	while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
	UCA0TXBUF = 0xD;
	wait(100);

	return;
}

void wait(int time){
	int a;
	for(a = 0; a<time; a++){}
}

void i2cWrite(int Adress, int Value){
    while (UCB0CTL1 & UCTXSTP);			//check if STOP is sent
	WriteMode = 1;						//important for the ISR procedure
	WriteRegister = Adress;
	TransmittedData = Value;
  	phase = 0;							//important for ISR procedure

	IE2 &= ~UCB0RXIE;					//clear RX interrupt-enable...
	IE2 |= UCB0TXIE;					//... and set TX interrupt-enable

    UCB0CTL1 |= UCTR + UCTXSTT;			//send START condition + address + Write
    __bis_SR_register(CPUOFF + GIE);	//go into a LPM and wait for ISR

	WriteMode = 0;
	return;
}

unsigned char i2cRequest(unsigned char Register){
		TransmittedData = Register;
		WriteMode = 0;						//important for ISR procedure
	  	phase = 0;							//important for ISR procedure
		IE2 &= ~UCB0RXIE;					//clear RX interrupt-enable
		IE2 |= UCB0TXIE;					//set TX interrupt-enable

	    while (UCB0CTL1 & UCTXSTP);			//ensure STOP condition was sent
	    UCB0CTL1 |= UCTR + UCTXSTT;			//send START condition + address + Write
	    __bis_SR_register(CPUOFF + GIE);	//go into a LPM and wait for ISR

	    while (UCB0CTL1 & UCTXSTP);			//ensure STOP was sent
	   	return ReceivedData;
}

void i2cSetup(int Adress){
	  P1SEL |= BIT6 + BIT7;						//declare P1.6 and P1.7 as I2c pins
	  P1SEL2|= BIT6 + BIT7;
	  UCB0CTL1 |= UCSWRST;						//stop UCB0
	  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		//configure UCB0
	  UCB0CTL1 = UCSSEL_2 + UCSWRST;
	  UCB0BR0 = 12;
	  UCB0BR1 = 0;
	  UCB0I2CSA = Adress;						//set sensor's I2C address
	  UCB0CTL1 &= ~UCSWRST;						//start UCB0
	  IE2 |= UCB0TXIE + UCB0RXIE;				//enable TX and RX interrupts
	  return;
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	if(WriteMode){
		if(phase == 0){
			UCB0TXBUF = WriteRegister;				//put the register address on the bus
			phase = 1;

		} else if (phase == 1){
			phase = 2;
			UCB0TXBUF = TransmittedData;			//put data on the bus

		}else if(phase == 2){
			IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);		//disable RX and TX interrupts
			UCB0CTL1 |= UCTXSTP;					//send STOP condition
			__bic_SR_register_on_exit(CPUOFF);		//exit LPM
		    WriteMode = 0;
		    phase = 0;
		}
	}else{
		//Read-mode
		if(phase == 0){
			UCB0TXBUF = TransmittedData;			//put register address on the bus
			phase = 1;

		}else if (phase == 1)
		{
			IE2 &= ~UCB0TXIE;						//disable TX interrupts
			IE2 |= UCB0RXIE;						//enable RX interrupts

			UCB0CTL1 &= ~UCTR;						//clear UCTR bit (means READ mode on next START condition)
		    UCB0CTL1 |= UCTXSTT;					//send START condition

		    while (UCB0CTL1 & UCTXSTT);				//wait for START to be sent
		    UCB0CTL1 |= UCTXSTP;					//send STOP condition and wait for the sensor-data
			phase = 2;

		}
		else if (phase == 2)
		{
			ReceivedData = UCB0RXBUF;				//read sensor-data from the bus
			IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);		//disable interrupts
			__bic_SR_register_on_exit(CPUOFF);		//exit LPM

		}

	}
}
void setupPWM(){

	DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	DCOCTL = CALDCO_1MHZ;

    //define "throttle"-positions
    throttleDown = 0.52 * pwmTimerLimit;
    throttleUp = 0.98 * pwmTimerLimit;
    throttleNeutral = 0.75 * pwmTimerLimit;

    P3DIR |= BIT0 + BIT2 + BIT3 + BIT5 + BIT6;				// TA0.2 + TA1.2 (motor east + motor west)
    P3SEL |= BIT0 + BIT2 + BIT3 + BIT5 + BIT6;

    //Setting up the PWMs:
    TA0CCR0 = pwmTimerLimit; 					//setting PWM frequency (TA0CCR0 contains the upper TA0 limit)
    TA0CCR1 = throttleDown;						//setting the duty-cycle to 52% initially
    TA0CCR2 = throttleDown;

    TA1CCR0 = pwmTimerLimit;
    TA1CCR1 = throttleDown;
    TA1CCR2 = throttleDown;

    TA0CCTL1 = OUTMOD_7;						//activate PWM set/reset
    TA0CCTL2 = OUTMOD_7;
    TA1CCTL1 = OUTMOD_7;
    TA1CCTL2 = OUTMOD_7;

    TA0CTL = TASSEL_2 | ID_3 | MC_1 | TACLR;	//source: SMCLK, divider:8, count to CCR0, clear Timer
    TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR;

}

void setMotorNorth(char throttle){
	TA0CCR1 = throttle;
}
void setMotorEast(char throttle){
	TA0CCR2 = throttle;
}
void setMotorSouth(char throttle){
	TA1CCR1 = throttle;
}
void setMotorWest(char throttle){
	TA1CCR2 = throttle;
}
