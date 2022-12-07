#include <Servo.h>

//SERVO
int pos = 0;
Servo servo1; 
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;


//BUTTON
const int greenbuttonPin = 47;  // the number of the pushbutton pin
int index = 3;
int greenbuttonState = HIGH;

const int redbuttonPin = 46;  // the number of the pushbutton pin
int redbuttonState = 0;

//RELAYS
int relayPin1 = 26;
int relayPin2 = 27;
int relayPin3 = 28;
int relayPin4 = 29;
int relayPin5 = 30;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo1.attach(37); //pin
  servo2.attach(38); //pin
  servo3.attach(39); //pin
  servo4.attach(40); //pin
  servo5.attach(41); //pin

  servo3.write(195);
  servo4.write(170);
  servo5.write(170);

  pinMode(greenbuttonPin, INPUT_PULLUP);
  //pinMode(redbuttonPin, INPUT_PULLUP);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  pinMode(relayPin5, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

  greenbuttonState = HIGH;
  greenbuttonState = digitalRead(greenbuttonPin);

  Serial.print("greenbuttonState :");
  Serial.println(greenbuttonState);

  delay(500);


  //Currently continuously loops, avoided using the button during the demo
  if (true) { 

    Serial.print("button pressed : ");
    Serial.println(index);

    switch(index){

      case 3:
      servo3.write(25); //different positions per scoop depending on how they were installed
      delay(8000);
      servo3.write(195);
      delay(8000);
      //vibrate();
      break;

      case 4:
      servo4.write(15);
      delay(8000);
      servo4.write(170);
      delay(8000);
      //vibrate();
      break;

      case 5:
      servo5.write(15);
      delay(8000);
      servo5.write(170);
      delay(8000);
      //vibrate();
      break;

      default:
      break;

    }

    index = index + 1;


    if(index > 5){
      index = 3;
    }
  }


}

void vibrate(){

  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);
  digitalWrite(relayPin5, HIGH);

  delay(5000);

  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  digitalWrite(relayPin3, LOW);
  digitalWrite(relayPin4, LOW);
  digitalWrite(relayPin5, LOW);

}

void move(int number, int pos){
  switch(number){
    case 1:
      servo1.write(pos);
      delay(2000);
      digitalWrite(relayPin1, HIGH);
      delay(5000);
      digitalWrite(relayPin1,LOW);
      break;

    case 2:
      servo2.write(pos);
      delay(2000);
      digitalWrite(relayPin2, HIGH);
      delay(5000);
      digitalWrite(relayPin2,LOW);
      break;

    case 3:
      servo3.write(pos);
      delay(2000);
      digitalWrite(relayPin3, HIGH);
      delay(5000);
      digitalWrite(relayPin3,LOW);
      break;

    case 4:
      servo4.write(pos);
      delay(2000);
      digitalWrite(relayPin4, HIGH);
      delay(5000);
      digitalWrite(relayPin4,LOW);
      break;

    case 5:
      servo5.write(pos);
      delay(2000);
      digitalWrite(relayPin5, HIGH);
      delay(5000);
      digitalWrite(relayPin5,LOW);
      break;

    default:
      break;
    }
}
