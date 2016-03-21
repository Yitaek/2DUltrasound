/* Generate a sin and ramp for the display 
   Use pin 10 for analog output pin
   Need 3.9k and 0.1 uF for LPF
*/

#include <Wire.h>

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

double yVal[100];
double xVal[100];
uint8_t i=0;

void setup() {

  setPwmFrequency(10,1);
  Serial.begin(9600);

  // Generate a sine wave
  for(int i=0; i<100; i++){
    // 1 Vpp wave, 50 Hz
    yVal[i] = (255/5) * (0.5 + 0.5 * sin(2*3.14/20*i));
  }

}

void loop() {
    for(int i=0; i<100; i++){
      analogWrite(10, yVal[i]);

      // 1000 Hz sampling rate
      delay(1);
    }
}


