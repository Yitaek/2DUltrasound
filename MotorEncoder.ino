#include <Stepper.h>
const int stepsPerRevolution = 1600; 
#define encoder0PinA  2
#define encoder0PinB  4

Stepper myStepper(stepsPerRevolution, 8, 9);
int forwardPin = 11;
int backwardPin = 12;
int buttonState = 0;
const int buttonPin = 10;
volatile unsigned int encoder0Pos = 0;
unsigned int zeroPos = 0;
int currentVal = 0;
int prevVal = 0;
int encoderVal = 0;

int inputPin = 3;
int dostep = 0;


void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(forwardPin,OUTPUT);
  pinMode(backwardPin,OUTPUT);
  digitalWrite(encoder0PinA, HIGH); 
  digitalWrite(encoder0PinB, HIGH); 
  attachInterrupt(0, doEncoder, CHANGE);
  pinMode(inputPin,INPUT);
  digitalWrite(inputPin,LOW);

  // put your setup code here, to run once:
 Serial.begin (9600);
 while(buttonState == LOW){ 
  Serial.println("Waiting...");
   buttonState = digitalRead(buttonPin);
 }
 delay(1000);
 buttonState = digitalRead(buttonPin);
  myStepper.setSpeed(200);
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

  }
  attachInterrupt(digitalPinToInterrupt(inputPin), increment, RISING);
   
}
boolean forward =0;
void loop() {

  if (dostep) {

    dostep = 0;
    if ((currentVal > prevVal && currentVal < zeroPos + 90) || currentVal < zeroPos - 90 ) { //moving forward
      while (encoder0Pos < currentVal + 4) {
        myStepper.step(1);
        
      }
      if(!forward){
        digitalWrite(backwardPin,LOW); 
      digitalWrite(forwardPin,HIGH);
      delay(4);
      digitalWrite(forwardPin,LOW);
      forward =1;
      }
      
      digitalWrite(backwardPin,LOW);
       prevVal = currentVal;
      currentVal = encoder0Pos;
   

      }
      else {
      while (encoder0Pos > currentVal - 4) {
        myStepper.step(-1);
       
      }
      prevVal = currentVal;
      currentVal = encoder0Pos;
      digitalWrite(forwardPin,LOW);
      digitalWrite(backwardPin,HIGH);
      forward = 0;
      
    }

}
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
 
}

void increment(){

   dostep = 1;

}

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

