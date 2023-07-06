#include <msp430.h> 
#define POT_CS_PIN BIT2   // P2.2 is used as Chip Select (CS) pin for the potentiometer
#define POT_CS_OUT P2OUT  // Port 2 output register for CS pin
#define POT_CS_DIR P2DIR  // Port 2 direction register for CS pin
#define SPI_SCLK_PIN BIT5  // P1.5 as SPI CLK
#define SPI_MOSI_PIN BIT7  // P1.7 as SPI MOSI
#define MCP_SS_PIN BIT2     // P2.2 as MCP4251 Chip select
#define MCP_WRITE_CMD 0x00 // Write to potentiometer

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

void spiWrite(uint8_t data) {
  while (!(IFG2 & UCB0TXIFG));  // Wait until TX buffer is ready
  UCB0TXBUF = data;             // Write data to TX buffer
}

unsigned char receiveByte() {
  while (!(IFG2 & UCA0RXIFG)); // Wait until byte is received
  return UCA0RXBUF; // Return received byte
}

void setResistance(uint16_t resistance) {
  uint8_t highByte = (resistance >> 8) & 0xFF; // Extract high byte of resistance
  uint8_t lowByte = resistance & 0xFF; // Extract low byte of resistance

  P1OUT &= ~MCP_SS_PIN;        // Select MCP4251 (pull SS pin low)

  spiWrite(MCP_WRITE_CMD);     // Send write command
  spiWrite(highByte);          // Send high byte of resistance
  spiWrite(lowByte);           // Send low byte of resistance

  P1OUT |= MCP_SS_PIN;         // Deselect MCP4251 (pull SS pin high)
}

void setWiper(float x) {
  uint8_t position = (uint8_t)(5 - x) * 51; //R_WB / R_AW = 1 / (5-x) and *51 for scaling to max 255

  P1OUT &= ~MCP_SS_PIN;        // Select MCP4251 (pull SS pin low)

  spiWrite(MCP_WRITE_CMD);     // Send write command
  spiWrite(position);          // Send wiper position

  P1OUT |= MCP_SS_PIN;         // Deselect MCP4251 (pull SS pin high)
}

void charge(float current_c) {
    P2DIR |= BIT0; //charge_disable
    P2OUT &= ~BIT0; //charge_disable = 0
    P2DIR |= BIT1; //change_state
    P2OUT |= BIT1; //change_state = 1
    int r_prog = 1000/curent_c; //Charging_current = 1000/Rprog
    setResistance(r_prog);
}

void discharge(float current_d) {
    P2DIR |= BIT0; //charge_disable
    P2OUT |= BIT0; //charge_disable = 1
    P2DIR |= BIT1; //change_state
    P2OUT &= ~BIT1; //change_state = 0
    setWiper(current_d); //setting wiper position
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
	  UCB0CTL1 |= UCSWRST;        // Enable SPI module, disable SPI during configuration
	  UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
	  UCB0CTL1 |= UCSSEL_2;       // Set SMCLK as the clock source for SPI
	  UCB0BR0 |= 0x02;            // Set SPI clock prescaler: SMCLK/2
	  UCB0BR1 = 0;

	  P1SEL |= SPI_SCLK_PIN + SPI_MOSI_PIN;   // Set P1.5 and P1.7 to SPI mode
	  P1SEL2 |= SPI_SCLK_PIN + SPI_MOSI_PIN;

	  P2DIR |= MCP_SS_PIN;        // Set MCP4251 SS pin as output
	  P2OUT |= MCP_SS_PIN;        // Set MCP4251 SS pin high

	  UCB0CTL1 &= ~UCSWRST;       // Enable SPI module

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
	       //Date din aplicatia de control
	       float curent_prag = receiveFloat();
	       float tensiune_prag = receiveFloat();
	       int iterations = receiveInt();
	       float current_set = receiveFloat();

	       while (iterations > 0) {
	         int voltage = readVoltage();
	         int current = readCurrent();

	         if (voltage <= tensiune_prag) {
	           charge(current_set);
	           voltage = readVoltage();
	           current = readCurrent();
	           }
	         } else if (voltage > tensiune_prag) {
	           discharge(current_set);
	           voltage = readVoltage();
	           current = readCurrent();
	         }

	         iterations--;
	       }

	
	return 0;
}
