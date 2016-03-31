/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pullups on A & B channel outputs
   turning on the pullups saves having to hook up resistors
   to the A & B channel outputs

*/

#define encoder0PinA  2
#define encoder0PinB  4
#include <Stepper.h>
#include <Wire.h>
volatile int currentVal = 0;
volatile int prevVal = -4;
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
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
double rVal[50] = {0 , 0.0204082 , 0.0408163 , 0.0612245 , 0.0816327 , 0.102041 , 0.122449 , 0.142857 , 0.163265 , 0.183673 , 0.204082 , 0.22449 , 0.244898 , 0.265306 , 0.285714 , 0.306122 , 0.326531 , 0.346939 , 0.367347 , 0.387755 , 0.408163 , 0.428571 , 0.44898 , 0.469388 , 0.489796 , 0.510204 , 0.530612 , 0.55102 , 0.571429 , 0.591837 , 0.612245 , 0.632653 , 0.653061 , 0.673469 , 0.693878 , 0.714286 , 0.734694 , 0.755102 , 0.77551 , 0.795918 , 0.816327 , 0.836735 , 0.857143 , 0.877551 , 0.897959 , 0.918367 , 0.938776 , 0.959184 , 0.979592 , 1};
uint8_t i = 0;

volatile unsigned int encoder0Pos = 0;
unsigned int encoderVal = 0;
unsigned int zeroPos = 0;
int angle = 0;
const int stepsPerRevolution = 1600;  // change this to fit the number of steps per revolution
// for your motor
volatile boolean dostep = 0;
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9);

int stepCount = 0;
const int buttonPin = 5;
//const int outputPin = 11;
const int inputPin = 3;
int buttonState = 0;
//int LEDpin = 10;
//volatile int state = LOW;
void setup() {
  //  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  setPwmFrequency(10, 1);
  Serial.begin (9600);
  myStepper.setSpeed(200);
  //myStepper.step(400);
  Serial.println("start");
  buttonState = digitalRead(buttonPin);
  stepCount++;
  while (buttonState == LOW) {
    myStepper.step(1);
    buttonState = digitalRead(buttonPin);
  }
  Serial.println (encoder0Pos, DEC);
  encoderVal = encoder0Pos;
  zeroPos = encoderVal - 256;
  currentVal = zeroPos;
  prevVal = currentVal - 4;
  delay(500);
  while (encoder0Pos > zeroPos) {
    myStepper.step(-1);
    angle = (encoder0Pos - zeroPos) * 0.35;
  }
  angle = encoder0Pos - zeroPos;
  angle = angle * 0.35;
  attachInterrupt(digitalPinToInterrupt(inputPin), increment, RISING);
}


void loop() {
// digitalWrite(LEDpin, state);
  if (dostep) {
    
    dostep = 0;
    if ((currentVal > prevVal && currentVal < zeroPos + 128) || currentVal < zeroPos - 128 ) { //moving forward
      while (encoder0Pos < currentVal + 4) {
        myStepper.step(1);
      }
      prevVal = currentVal;
      currentVal = encoder0Pos;
      angle = encoder0Pos - zeroPos;
      angle = angle * 0.35;
      for (int i = 0; i < 50; i++) {
        xVal[i] = (255 / 5) * (1 + rVal[i] * cos(2 * 3.14 / 360 * angle));
        yVal[i] = (255 / 5) * (1 + rVal[i] * sin(2 * 3.14 / 360 * angle));

        analogWrite(10, yVal[i]);
        analogWrite(11, xVal[i]);

      }
    }
    else {
      while (encoder0Pos > currentVal - 4) {
        myStepper.step(-1);
      }
      currentVal = encoder0Pos;
      angle = encoder0Pos - zeroPos;
      angle = angle * 0.35;
      for (int i = 0; i < 50; i++) {
        xVal[i] = (255 / 5) * (1 + rVal[i] * cos(2 * 3.14 / 360 * angle));
        yVal[i] = (255 / 5) * (1 + rVal[i] * sin(2 * 3.14 / 360 * angle));

        analogWrite(10, yVal[i]);
        analogWrite(11, xVal[i]);

      }
    }
   
  }
}




void increment() {
  dostep = 1;

}
void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }

  // Serial.println (encoder0Pos, DEC);
}

/* See this expanded function to get a better understanding of the
 * meanings of the four possible (pinA, pinB) value pairs:
 */
void doEncoder_Expanded() {
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }

  }

}

/*  to read the other two transitions - just use another attachInterrupt()
in the setup and duplicate the doEncoder function into say,
doEncoderA and doEncoderB.
You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/

