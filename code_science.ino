#include <Servo.h>
#include <ezButton.h>

//VICTOR
int speed = 0; //[-100,100] 100 is forward and -100 is reverse
int drive_pin = 1; //pin
int pwm_high_time = 0;
int pwn_low_time = 1000; // en micro secondes

//LIMIT SWITCH
ezButton limitSwitchTop(2); //pin
bool isTopReached = false;

ezButton limitSwitchBot(3); //pin
bool isBotReached = false;

//SERVO
int pos = 0;
Servo servo1; 
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

//SOLENOID
int solenoid1_pin = 9;
int solenoid2_pin = 10;
int solenoid3_pin = 11;
int solenoid4_pin = 12;
int solenoid5_pin = 13;

//RELAYS
int relayPin1 = 26;
int relayPin2 = 27;
int relayPin3 = 28;
int relayPin4 = 29;
int relayPin5 = 30;


void setup() {
  Serial.begin(9600);

  //VICTOR
  pinMode(drive_pin, OUTPUT); 
  digitalWrite(drive_pin, LOW);
  pwm_high_time = 5 * speed + 1500; //en micro secondes

  //LIMIT SWITCH 
  limitSwitchTop.setDebounceTime(50);
  limitSwitchBot.setDebounceTime(50);

  //SERVO
  servo1.attach(4); //pin
  servo2.attach(5); //pin
  servo3.attach(6); //pin
  servo4.attach(7); //pin
  servo5.attach(8); //pin

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);


  //SOLENOID
  pinMode(solenoid1_pin, OUTPUT);
  pinMode(solenoid2_pin, OUTPUT);
  pinMode(solenoid3_pin, OUTPUT);
  pinMode(solenoid4_pin, OUTPUT);
  pinMode(solenoid5_pin, OUTPUT);

  //RELAYS
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  pinMode(relayPin5, OUTPUT);


}

void loop() {
  move(-50, "down"); //go down half speed
  scoop(1);
  move(50, "up"); //go up half speed 
  addWater(1);
  mix(1);
  checkStrip(1);

  //repeat for all scoops

}

void limitSwitch(){
 // EZ SWITCH LIBRARY TUTORIAL https://arduinogetstarted.com/tutorials/arduino-limit-switch

  if(limitSwitchTop.isPressed())
    isTopReached = true;
  

  if(limitSwitchTop.isReleased())
    isTopReached = false;
  

  if(limitSwitchBot.isPressed())
    isBotReached = true;
  

  if(limitSwitchBot.isReleased())
    isBotReached = false;
  

  //FOR LIMIT SWITCH TESTING
  int state_top = limitSwitchTop.getState();
  int state_bot = limitSwitchBot.getState();
  if(state_top == HIGH || state_bot == HIGH)
    Serial.println("NOTHING SO FAR");
  else
    Serial.println("A LIMIT SWITCH HAS BEEN TOUCHED");

}


void move(int speed, char direction){
 // VICTOR https://github.com/hatsunearu/arduino-victor888-controller/blob/master/arduino-victor888-controller.ino
 // VICTOR ALSO, LESS RELEVANT BUT SAME FORMULA https://www.chegg.com/homework-help/questions-and-answers/code-done-using-c--project-utilizing-arduino-uno-cim-motor-victor-spx-speed-controller-cod-q62962540

  if(!isTopReached && direction == "up"){
    pwm_high_time = 5 * speed + 1500; //en micro secondes
    digitalWrite(drive_pin, HIGH);
    delayMicroseconds(pwm_high_time);
    digitalWrite(drive_pin, LOW);
    delayMicroseconds(pwn_low_time);
  }

  if(isTopReached && direction == "up")
    speed = 0;
  

  if(!isBotReached && direction == "down"){
    pwm_high_time = 5 * speed + 1500; //en micro secondes
    digitalWrite(drive_pin, HIGH);
    delayMicroseconds(pwm_high_time);
    digitalWrite(drive_pin, LOW);
    delayMicroseconds(pwn_low_time);
  }

  if(isBotReached && direction == "down")
    speed = 0;

}

void scoop(int number){
 // https://automaticaddison.com/how-to-control-a-servo-motor-using-arduino/

  if(isBotReached){

    //pour dépasser plus que 90 degré pour un bigger scoop
      switch(number){
        case 1:
          servo1.write(100);
          break;

        case 2:
          servo2.write(100);
          break;

        case 3:
          servo3.write(100);
          break;

        case 4:
          servo4.write(100);
          break;

        case 5:
          servo5.write(100);
          break;
        
        default:
          break;
      }    

      delay(3000); //attendre 3 secondes avant de scoop, for the dirt to settle

      //scoop jusqu'à revenir à position initiale 
      switch(number){
        case 1:
          servo1.write(0);
          break;

        case 2:
          servo2.write(0);
          break;

        case 3:
          servo3.write(0);
          break;

        case 4:
          servo4.write(0);
          break;

        case 5:
          servo5.write(0);
          break;
        
        default:
          break;
      }
      delay(15);
    
  }
}

void addWater(int number){
  // https://bc-robotics.com/tutorials/controlling-a-solenoid-valve-with-arduino/

  switch(number){
    case 1:
      digitalWrite(solenoid1_pin, HIGH);
      delay(5000); //Wait 5 Seconds
      digitalWrite(solenoid1_pin, LOW);
      break;

    case 2:
      digitalWrite(solenoid2_pin, HIGH);
      delay(5000); //Wait 5 Seconds
      digitalWrite(solenoid2_pin, LOW);
      break;

    case 3:
      digitalWrite(solenoid3_pin, HIGH);
      delay(5000); //Wait 5 Seconds
      digitalWrite(solenoid3_pin, LOW);
      break;

    case 4:
      digitalWrite(solenoid4_pin, HIGH);
      delay(5000); //Wait 5 Seconds
      digitalWrite(solenoid4_pin, LOW);
      break;

    case 5:
      digitalWrite(solenoid5_pin, HIGH);
      delay(5000); //Wait 5 Seconds
      digitalWrite(solenoid5_pin, LOW);
      break;
    
    default:
      break;
  }
}

void checkStrip(int number){
 // https://automaticaddison.com/how-to-control-a-servo-motor-using-arduino/
 // https://www.makerguides.com/servo-arduino-tutorial/

  //va a 45 degré pour le permettre de lire les strips
    switch(number){
      case 1:
        servo1.write(45);
        break;

      case 2:
        servo2.write(45);
        break;

      case 3:
        servo3.write(45);
        break;

      case 4:
        servo4.write(45);
        break;

      case 5:
        servo5.write(45);
        break;
      
      default:
        break;
    }
    delay(15); //donne le temps pour reach la position 
  

    delay(10000); //10 secondes, nous donner le temps de lire la strip

  //scoop jusqu'à revenir à position initiale 
    switch(number){
      case 1:
        servo1.write(0);
        break;

      case 2:
        servo2.write(0);
        break;

      case 3:
        servo3.write(0);
        break;

      case 4:
        servo4.write(0);
        break;

      case 5:
        servo5.write(0);
        break;
      
      default:
        break;
    }
    delay(15);
  

}

void mix(int number){
  switch(number){
    case 1:
      digitalWrite(relayPin1, HIGH);
      delay(5000);
      digitalWrite(relayPin1, LOW);
      break;
    
    case 2:
      digitalWrite(relayPin2, HIGH);
      delay(5000);
      digitalWrite(relayPin2, LOW);
      break;

    case 3:
      digitalWrite(relayPin3, HIGH);
      delay(5000);
      digitalWrite(relayPin3, LOW);
      break;
    
    case 4:
      digitalWrite(relayPin4, HIGH);
      delay(5000);
      digitalWrite(relayPin4, LOW);
      break;

    case 5:
      digitalWrite(relayPin5, HIGH);
      delay(5000);
      digitalWrite(relayPin5, LOW);
      break;

    default:
      break;
  }
}
