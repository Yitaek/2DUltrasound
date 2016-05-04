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
//volatile double angle = 0;
int inputPin = 3;
int dostep = 0;
//volatile double xVal;
//volatile double yVal;

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
     //delayMicroseconds(100);
    //angle = (encoder0Pos - zeroPos) * 0.35;
  }
  attachInterrupt(digitalPinToInterrupt(inputPin), increment, RISING);
   
}
boolean forward =0;
void loop() {
  // put your main code here, to run repeatedly:
// myStepper.step(-10);
 //delay(100);
  if (dostep) {
    //Serial.println("step");
    //digitalWrite(clockpin, HIGH);
    dostep = 0;
    if ((currentVal > prevVal && currentVal < zeroPos + 90) || currentVal < zeroPos - 90 ) { //moving forward
      while (encoder0Pos < currentVal + 4) {
        myStepper.step(1);
        
      // Serial.println(encoder0Pos);
       // Serial.println("Forward");
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
     // angle = encoder0Pos - zeroPos;
   //   angle = angle * .35;
      //angle = angle*
      //xVal = (255 / 5) * (1 + cos(3.14/180*angle));
    //  yVal = (255 / 5) * (1 + sin(angle*3.14/180));
    //   setPwmFrequency(5, 1);
 // setPwmFrequency(6,1);
     // analogWrite(5, xVal);
    //  analogWrite(6, yVal);
 //      setPwmFrequency(5, 64);
 // setPwmFrequency(6,64);

      }
      else {
      while (encoder0Pos > currentVal - 4) {
        myStepper.step(-1);
        //Serial.println(encoder0Pos);
//              angle = encoder0Pos - zeroPos;
//              Serial.println(angle);
//      angle = angle *.35;
//      //angle = angle*
//      xVal = (255 / 5) * (1 + cos(angle*3.14/180));
//      yVal = (255 / 5) * (1 + sin(angle*3.14/180));
//      analogWrite(5, xVal);
//      analogWrite(6, yVal);
//        Serial.println(yVal);
//        Serial.println(angle);
      }
      prevVal = currentVal;
      currentVal = encoder0Pos;
      digitalWrite(forwardPin,LOW);
      digitalWrite(backwardPin,HIGH);
      forward = 0;
      //angle = encoder0Pos - zeroPos;
     // angle = angle * 0.35;
   
     // xVal = (255 / 5) * (1 + cos(2 * 3.14 / 360 * -1 * angle));
      //yVal = (255 / 5) * (1 + sin(2 * 3.14 / 360 * -1 * angle));
 //         setPwmFrequency(5, 1);
 // setPwmFrequency(6,1);
 //     analogWrite(5, xVal);
 //     analogWrite(6, yVal);
  //     setPwmFrequency(5, 64);
  //setPwmFrequency(6,64);
    }
// Serial.println("Done");
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
 // angle = encoder0Pos - zeroPos;
 //   angle = angle * 0.35;
//      //Serial.println(angle);
   // xVal = 255/5*(1+cos(2 * 3.14 / 360 * -1 * angle));
 //  analogWrite(5, xVal);
 // analogWrite(6,xVal);

  // Serial.println (encoder0Pos, DEC);
}

void increment(){
  //Serial.println("here");
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

