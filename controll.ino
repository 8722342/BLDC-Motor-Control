// Define variable storage
unsigned char ADC_value; // PWM threshold is ADC result
unsigned char LastSensor; // last read motor sensor data
unsigned char DriveWord; // six bit motor drive data

// Define I/O
#define OffMask 0b11010101
#define DrivePort PORTC
#define DrivePortTris TRISC
#define SensorMask 0b00000111
#define SensorPort PORTE
#define DirectionBit 1

void setup() {
  // Initialize I/O ports and peripherals
  pinMode(DirectionBit, INPUT);
  DrivePort = 0; // all drivers off
  DrivePortTris = 0; // set motor drivers as outputs
  TRISA = 0b00000011; // A/D on RA0, Direction on RA1, Motor sensors on RE<2:0>
  OPTION_REG = 0b11010111; // Timer0: Fosc, 1:2
  ADCON1 = 0b00001110; // ADC left justified, AN0 only
  ADCON0 = 0b11000001; // ADC clock from int RC, AN0, ADC on
  ADCON0 |= (1 << 1); // Start ADC
  
  LastSensor = 0; // initialize last sensor reading
  Commutate(); // determine present motor position
  ADC_value = 0; // start speed control threshold at zero until first ADC reading
}

void loop() {
  ReadADC(); // get the speed control from the ADC
  
  if (ADC_value == 0xFF) { // if ADC is 0xFF we're at full speed
    Drive(); // continue
  } else {
    PWM(); // add timer 0 to ADC for PWM
  }
}

void ReadADC() {
  // If the ADC is ready then read the speed control potentiometer
  // and start the next reading
  if (ADCON0 & (1 << 1)) { // is ADC ready?
    return; // no - return
  }
  
  ADC_value = ADRESH; // get ADC result
  ADCON0 |= (1 << 1); // restart ADC
}

void Commutate() {
  unsigned char sensorData = SensorPort & SensorMask; // retain only the sensor bits
  unsigned char motionSensed = sensorData ^ LastSensor; // test if motion sensed
  
  if (motionSensed == 0) {
    return; // no change - back to the PWM loop
  }
  
  LastSensor = sensorData; // replace last sensor data with current
  
  if (bitRead(PORTA, DirectionBit) == 0) { // test direction bit
    // forward commutation
    DriveWord = pgm_read_byte_near(FwdTable + sensorData);
  } else {
    // reverse commutation
    DriveWord = pgm_read_byte_near(RevTable + sensorData);
  }
}

void Drive() {
  DrivePort = DriveWord; // enable motor drivers
  Commutate(); // test for commutation change
}

void PWM() {
  unsigned int timerAddition = ADC_value + TMR0; // add ADC to current timer0
  if (timerAddition > 255) { // test if ADC + timer0 resulted in carry
    DriveWord &= OffMask; // no carry - suppress high drivers
  }
  
  Drive(); // continue
}

const unsigned char FwdTable[] PROGMEM = {
  0b00000000, // invalid
  0b00010010, // phase 6
  0b00001001, // phase 4
  0b00011000, // phase 5
  0b00100100, // phase 2
  0b00000110, // phase 1
  0b00100001, // phase 3
  0b00000000  // invalid
};

const unsigned char RevTable[] PROGMEM = {
  0b00000000, // invalid
  0b00100001, // phase /6
  0b00000110, // phase /4
  0b00100100, // phase /5
  0b00011000, // phase /2
  0b00001001, // phase /1
  0b00010010, // phase /3
  0b00000000  // invalid
};
