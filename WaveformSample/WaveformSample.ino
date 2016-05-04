/* Generate a sin and ramp for the display 
   Use pin 10 for analog output pin
   Need 3.9k and 0.1 uF for LPF
*/

#include <Wire.h>


unsigned long time1, time2;

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

//double rVal[50] = {0 , 0.0204082 , 0.0408163 , 0.0612245 , 0.0816327 , 0.102041 , 0.122449 , 0.142857 , 0.163265 , 0.183673 , 0.204082 , 0.22449 , 0.244898 , 0.265306 , 0.285714 , 0.306122 , 0.326531 , 0.346939 , 0.367347 , 0.387755 , 0.408163 , 0.428571 , 0.44898 , 0.469388 , 0.489796 , 0.510204 , 0.530612 , 0.55102 , 0.571429 , 0.591837 , 0.612245 , 0.632653 , 0.653061 , 0.673469 , 0.693878 , 0.714286 , 0.734694 , 0.755102 , 0.77551 , 0.795918 , 0.816327 , 0.836735 , 0.857143 , 0.877551 , 0.897959 , 0.918367 , 0.938776 , 0.959184 , 0.979592 , 1};
double rVal = 1; 
uint8_t i = 0;

double angle[10] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90}; 

void setup() {

  setPwmFrequency(10,1);
  Serial.begin(9600);

  // Generate a sine wave

  // time1 = micros();
         
       for (int i = 0; i < 10; i++) {
         // time1 = micros();
        xVal[i] = (255 / 5) * (1 + rVal * cos(2 * 3.14 / 360 * angle[i]));
        yVal[i] = (255 / 5) * (1 + rVal * sin(2 * 3.14 / 360 * angle[i]));

        
        analogWrite(10, yVal[i]);
        analogWrite(11, xVal[i]);
        
        delay(1);
        
          
      }
      

}

void loop() {

}


