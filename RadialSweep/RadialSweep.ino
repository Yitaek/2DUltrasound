/* Simulate a radial sweep for the display */

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

double yVal[50];
double xVal[50];
double rVal[50] = {0 ,0.0204082 ,0.0408163 ,0.0612245 ,0.0816327 ,0.102041 ,0.122449 ,0.142857 ,0.163265 ,0.183673 ,0.204082 ,0.22449 ,0.244898 ,0.265306 ,0.285714 ,0.306122 ,0.326531 ,0.346939 ,0.367347 ,0.387755 ,0.408163 ,0.428571 ,0.44898 ,0.469388 ,0.489796 ,0.510204 ,0.530612 ,0.55102 ,0.571429 ,0.591837 ,0.612245 ,0.632653 ,0.653061 ,0.673469 ,0.693878 ,0.714286 ,0.734694 ,0.755102 ,0.77551 ,0.795918 ,0.816327 ,0.836735 ,0.857143 ,0.877551 ,0.897959 ,0.918367 ,0.938776 ,0.959184 ,0.979592 ,1};

double angles[90] = {-45 ,-35 ,-25 ,-15 ,-5 ,5 ,15 ,25 ,35 ,45};



uint8_t i=0;

void setup() {

  setPwmFrequency(10,1);
  Serial.begin(9600);
  
  // Radius 0.1 - 1 Vpp covers the screen 

}

void loop() {
  for(int n=0; n<10; n++){
    for(int i=0; i<50; i++){
        xVal[i] = (255/5) * (rVal[i]/2 + rVal[i] * cos(2*3.14/360*angles[n]));
        yVal[i] = (255/5) * (rVal[i]/2 + rVal[i] * sin(2*3.14/360*angles[n]));
      
        analogWrite(10, yVal[i]);
        analogWrite(11, xVal[i]);
  
        // 1000 Hz sampling rate
        delay(1);
      }
    
    delay(10);
  }
}


