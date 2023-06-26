#include <msp430.h> 
#define POT_CS_PIN BIT2   // P2.2 is used as Chip Select (CS) pin for the potentiometer
#define POT_CS_OUT P2OUT  // Port 2 output register for CS pin
#define POT_CS_DIR P2DIR  // Port 2 direction register for CS pin

double read_voltage() {
  ADC10CTL0 &= ~ENC;              // Disable ADC conversion
  ADC10CTL1 &= ~(INCH_0 + INCH_4); // Clear INCH bits
  ADC10CTL1 |= INCH_4;             // Set INCH to P1.4 (voltage measurement)
  ADC10CTL0 |= ENC + ADC10SC;      // Enable conversion and start ADC conversion

  while (ADC10CTL1 & ADC10BUSY) {} // Wait for conversion to complete

  int result_div = ADC10MEM;    //salvez rezultatul preliminar obtinut ca valoare intre 0 si 1023

  double voltage = 2.0 * result_div * 3.3 / 1023; //inmultesc cu 2 rezultatul obtinut si apoi scalez cu factorul de conversie

  return voltage;
}



double read_current() {
  ADC10CTL0 &= ~ENC;              // Disable ADC conversion
  ADC10CTL1 &= ~(INCH_0 + INCH_4); // Clear INCH bits
  ADC10CTL1 |= INCH_0;             // Set INCH to P1.0 (current measurement)
  ADC10CTL0 |= ENC + ADC10SC;      // Enable conversion and start ADC conversion

  while (ADC10CTL1 & ADC10BUSY) {} // Wait for conversion to complete

  int result_bits = ADC10MEM;   //salvez rezultatul preliminar obtinut ca valoare intre 0 si 1023

  double current = result_bits * 3.3 / 1023; //convertesc la Amperi inmultind cu factorul de conversie

  return current;
}

void chargeBattery(int chargingCurrent) {
  // Code for charging the battery with the specified charging current
  // Use the chargingCurrent value to control the charging process
}

void dischargeBattery(int dischargingCurrent) {
  // Code for discharging the battery with the specified discharging current
  // Use the dischargingCurrent value to control the discharging process
}

void sendNumber(int number) {
  char buffer[16];
  sprintf(buffer, "%d", number);       // Convert number to string
  sendString(buffer);
}

float receiveFloat() {
  unsigned char buffer[4];
  buffer[0] = receiveByte();
  buffer[1] = receiveByte();
  buffer[2] = receiveByte();
  buffer[3] = receiveByte();

  float number;
  memcpy(&number, buffer, sizeof(float));
  return number;
}

int receiveInt() {
  unsigned char buffer[4];
  buffer[0] = receiveByte();
  buffer[1] = receiveByte();
  buffer[2] = receiveByte();
  buffer[3] = receiveByte();

  int number;
  memcpy(&number, buffer, sizeof(int));
  return number;
}


int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	  ADC10CTL1 = INCH_4 + ADC10DIV_3; // Use P1.4 for voltage measurement, ADC10CLK/4
	  ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE; // Vcc/Vss as reference, 64 ADC10CLK cycles, ADC on, enable ADC interrupt
	  ADC10AE0 |= BIT4; // Enable P1.4 as ADC input

	  // Configure P1.0 for current measurement
	  P1DIR &= ~BIT0;
	  ADC10CTL1 |= INCH_0; // Use P1.0 for current measurement
	  ADC10AE0 |= BIT0;    // Enable P1.0 as ADC input

	  // Configure SPI
	  UCA0CTL1 = UCSWRST;            // Put SPI module in reset state
	  UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
	  UCA0CTL1 |= UCSSEL_2;          // Set SMCLK as the clock source
	  UCA0BR0 = 0x02;                // Configure SPI clock frequency (replace with appropriate value)
	  UCA0BR1 = 0;
	  UCA0CTL1 &= ~UCSWRST;          // Enable SPI module

	  POT_CS_DIR |= POT_CS_PIN;     // Set CS pin as output

	  //UART
	    P1SEL |= BIT1 + BIT2; // P1.1 = RXD, P1.2=TXD
	    P1SEL2 |= BIT1 + BIT2; //P1.1 = RXD, P1.2=TXD

	    UCA0CTL1 |= UCSWRST; // Enable software reset

	    UCA0CTL0 = 0; // No parity, LSB first, 8-bit data, 1 stop bit
	    UCA0CTL1 |= UCSSEL_2; // Use SMCLK as the clock source
	    UCA0BR0 = 104; // Set the baud rate to 9600
	    UCA0BR1 = 0;
	    UCA0MCTL = UCBRS_1; // Modulation UCBRSx = 1
	    UCA0CTL1 &= ~UCSWRST; // Clear software reset

	    IE2 |= UCA0RXIE; // Enable UART receive interrupt

	    while (1) {
	       // Receive data from Python app
	       float curent_prag = receiveFloat();
	       float tensiune_prag = receiveFloat();
	       int iterations = receiveInt();

	       while (iterations > 0) {
	         int voltage = readVoltage();
	         int current = readCurrent();

	         // Perform actions based on voltage and current values
	         if (voltage >= curent_prag) {
	           chargeBattery(100);
	         } else if (voltage <= DISCHARGE_THRESHOLD) {
	           dischargeBattery(50);
	         }

	         iterations--;
	       }

	
	return 0;
}
