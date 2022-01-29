#include <Wire.h>
#define SLAVE_ADDRESS 0x04       // I2C address for Arduino
#define LED 13                   // Built-in LED
int i2cData = 4;                 // the I2C data received
int i2cData3 = 3;
int i2cData2 = 2;
int flag = 0;




/*STEPPER+SERVO_DEP*/
#include <Servo.h>
#include <Stepper.h>
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 9, 10, 11, 12);
Servo myservo;


//**************************++++++++++++++++++++++BUT1 2 LED 1 2 3 + BUTSTATES 1 2**********+++++++++++++++++++++***************

const int LED1 = 7;
const int LED2 = 6;
const int LED3 = 41;
int BUTTONstate1 = 0;
int BUTTONstate2 = 0;
int photoresistance=A7;
int valeur;
//******************************************************************************************************************************


//*****&&&&&&&&&&&&*********++++MODE VARIABLES++++*******&&&&&&&&&&&*********
int triggerPin = 2;
int echoPin = 3;
int buzzerPin = 8;
float vitesse = 0.0340;
float temps;
float distance;
int redPin = 25; //red pin of RGB led plugged into digital pin 9
 int greenPin = 27; // green pin plugged into digital pin 10
 int bluePin = 29;//and so on
 int buttonstate_mode2 = 23;
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù
int potpiin = A10;




/*DHT11 CHUNK*/ /*********************************/
#include <dht.h>


#define dht_apin A0 // Analog Pin sensor is connected to

dht DHT;

int dht_val = 0;
/****************************************************************/
int func1_val = 0;
int func1_val2 = 0;

// Handle reception of incoming I2C data
void receiveData(int byteCount) {
  while (Wire.available()) {
    i2cData = Wire.read();
    if (i2cData == 1) {

      flag = 1;
    }
    else if(i2cData == 0){

      flag = 2;
    }
  }
}
// Handle request to send I2C data
void sendData() {
  if(flag==1){
    Wire.write(func1_val);

  }else if(flag==2){
    Wire.write(dht_val);
  }

}





//*****&&&&&&&&&&&&*********++++MODE 1 VARIABLES AND FUNCT++++*******&&&&&&&&&&&*********
//LED BLINK MODE 1
//LED 1 PIN 7+ GROUND + RESISTANCE

void ledblink_MODE1(){
    digitalWrite(LED2, 1);
    delay(700);
    digitalWrite(LED2, 0);
    delay(700);
  }

//*****&&&&&&&&&&&&*********++++MODE 3 VARIABLES++++*******&&&&&&&&&&&*********
void ledbuttonControl_MODE3(){
    buttonstate_mode2 = digitalRead(23);
    if(buttonstate_mode2==1){
        digitalWrite(LED2, 1);
      }else{
        digitalWrite(LED2, 0);
        }
  }
//*****&&&&&&&&&&&&*********++++MODE 2 VARIABLES++++*******&&&&&&&&&&&*********


void lcdPrintButState_MODE2(){
    buttonstate_mode2 = digitalRead(23);
    if(buttonstate_mode2==1){
        //SET_FLAG_BUTTONSTATE
      }else{
        //SET_FLAG_BUTTONSTATE
        }
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



//*****&&&&&&&&&&&&*********++++MODE 4 VARIABLES++++*******&&&&&&&&&&&*********

const int X_pin = A3; // analog pin connected to X output
const int Y_pin = A4; // analog pin connected to Y output

int YPIN; //declaring YPIN an integer for later
int XPIN;
int SWPIN;


void setColor(int red, int green, int blue) //sets up format for colors
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

void RGB_MODE4_MAIN() {
  int YPIN = analogRead(Y_pin);  //sets the number thats comes out of analogRead(Y_pin); equal to a new variable
  int XPIN = analogRead(X_pin);
  //int SWPIN = digitalRead(SW_pin);
  delay(100);

  if(YPIN == 0) {setColor(255, 255, 255); delay(100);} //if YPIN is equal to 0, set the color
  else {setColor(0, 0, 0);}

  if(YPIN > 1000) {setColor(225, 0, 225); delay(100);}
    else {setColor(0, 0, 0);}

  if(XPIN == 0) {setColor(0, 0, 225); delay(100);}
    else {setColor(0, 0, 0);}

  if(XPIN > 1000) {setColor(0, 225, 0); delay(100);}
    else {setColor(0, 0, 0);}

   if(SWPIN == LOW) {setColor(80, 20, 0); delay(100);} //if SWPIN is low set color
    else {setColor(0, 0, 0);}

}
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 5 VARIABLES pot DELAY BLINK++++*******&&&&&&&&&&&*********
void PotLedBlindDelay_MODE_5(){
  int potValue = analogRead (A5);
  digitalWrite (LED2, HIGH);
  digitalWrite (LED1, HIGH);
  delay (potValue);
  digitalWrite (LED2, LOW);
  digitalWrite (LED1, LOW);
  delay (potValue);
  }

//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 6 VARIABLES PWM LED BRIGHTNESS POT++++*******&&&&&&&&&&&*********
void PWM_Led_MODE_6(){
  int inputValue = analogRead(A5);
  int outputValue = map(inputValue, 0, 1023, 0, 255);
  analogWrite(LED2, outputValue);
  delay(100);

  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 7 VARIABLES POT DC MOTOR SPEED++++*******&&&&&&&&&&&*********
void dc_motor_speed_MODE_7(){
  int motor1pin1 = 31;
  int motor1pin2 = 33;
  int enb = 4;
  int inputValue = analogRead(A5);
  int outputValue = map(inputValue, 0, 1023, 255, 0);
  //OUTPUTVALUE FLAG
  digitalWrite(motor1pin1, 1);
  digitalWrite(motor1pin2, 0);
  analogWrite(enb, outputValue);

  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



//*****&&&&&&&&&&&&*********++++MODE 8 VARIABLES Servo lcd pot++++*******&&&&&&&&&&&*********
void servopotlcd_MODE_8(){
  int inputValue = analogRead(A5);
  int outputValue = map(inputValue, 0, 1023, 0, 180);
  //OUTPUTVALUE FLAG
  myservo.write(outputValue);
  delay(15);
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



//*****&&&&&&&&&&&&*********++++MODE 9 STEPPER PROB++++*******&&&&&&&&&&&*********
void stepbystep_MODE_9(){
  myStepper.step(stepsPerRevolution);
  delay(500);
  }

//*****&&&&&&&&&&&&*********++++MODE 10 STEPPER KNOB PROB++++*******&&&&&&&&&&&*********
void Knob_STEPPER_MODE10(){
  int val = analogRead(A5);
  int previous = 0;
  // move a number of steps equal to the change in the
  // sensor reading
  myStepper.step(val - previous);

  // remember the previous value of the sensor
  previous = val;
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù
//*****&&&&&&&&&&&&*********++++MODE 11 Joystick SERVO++++*******&&&&&&&&&&&*********
void JoyServoControl_MODE_11(){
  delay(200);
  int joystickXVal = analogRead(X_pin);
  //joystickXVal INPUT

  //(joystickXVal+520)/10 OUTPUT

  myservo.write((joystickXVal+520)/10);

  }

//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 13 HC05 LED ON OFF APP++++*******&&&&&&&&&&&*********
char junk;
String inputString="";
void HC05_LEDonoff_MODE_13(){
 
    // put your main code here, to run repeatedly:
 if(Serial.available()>0)
   {
      char data= Serial.read(); // reading the data received from the bluetooth module
      switch(data)
      {
        case 'a':
        //FLAG INFO : LED ON
        digitalWrite(LED1, HIGH);break; // when a is pressed on the app on your smart phone
        case 'b':
        //FLAG INFO : LED OFF
        digitalWrite(LED1, LOW);break; // when d is pressed on the app on your smart phone
        default : break;
      }
   }
   delay(50);

  }

//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 14 ULTRASON ++++*******&&&&&&&&&&&*********
void ultrason_lcd_MODE_14(){
  delay(200);
  digitalWrite(triggerPin, 1);
  delayMicroseconds(10);
  digitalWrite(triggerPin, 0);

  temps = pulseIn(echoPin, 1);

  distance = (temps / 2) * vitesse;
  //FLAG : DISTANCE
   if(distance <= 15){
        digitalWrite(LED2, 1);
        digitalWrite(LED1, 1);
        tone(buzzerPin, 1000);
      }else if( (distance<=30) && (distance>15) ){
        noTone(buzzerPin);
        digitalWrite(LED2, 0);
        digitalWrite(LED1, 1);
    } else {
        digitalWrite(LED1, 0);
        digitalWrite(LED2, 0);
        noTone(buzzerPin);
      }

  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



//*****&&&&&&&&&&&&*********++++MODE 15 INFRAROUGE DETECTION D OBSTACLE++++*******&&&&&&&&&&&*********
int cap_infra = 37;
void infra_obst_detec_MODE_15(){
    int detection = digitalRead(cap_infra);
    if (detection==0){
        //FLAG OBSTACLE PRESENT
        digitalWrite(LED1, 1);
      }
    if (detection == 1){
        //FLAG PAS D OBSTACLES
        digitalWrite(LED1, 0);

      }
   delay(1000);
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 16 PIR LED ON OFF++++*******&&&&&&&&&&&*********
int state;
int val = 0;
int sensor = 39;
void PIR_MODE_16(){

    int val = digitalRead(sensor);   // read sensor value
  if (val == HIGH) {           // check if the sensor is HIGH
    digitalWrite(LED1, HIGH);   // turn LED ON
    delay(100);                // delay 100 milliseconds

    if (state == LOW) {
      //FLAG INFO : Motion detected!
      state = HIGH;       // update variable state to HIGH
    }
  }
  else {
      digitalWrite(LED1, LOW); // turn LED OFF
      delay(200);             // delay 200 milliseconds

      if (state == HIGH){
        //FLAG INFO Motion stopped!
        state = LOW;       // update variable state to LOW
    }
  }
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



//*****&&&&&&&&&&&&*********++++MODE 17 CAPTEUR DE SON LED ON OFF++++*******&&&&&&&&&&&*********
void CAP_Son_MODE_17(){
    int capval = analogRead(A7);
    digitalWrite(LED1, 1);
    delay(capval);
    digitalWrite(LED1, 0);
    delay(capval);
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù


//*****&&&&&&&&&&&&*********++++MODE 18 DETECC DE FLAMMES ++++*******&&&&&&&&&&&*********
void flamme_detec_MODE_18(){
  int analogVal = analogRead(potpiin);
  if (analogVal<=11){ // le capteur KY-026 détecte un feu
  digitalWrite(LED1, HIGH); // LED s’allume et le buzzer sonne
} else { // sinon
digitalWrite(LED1, LOW); // LED s’éteint et le buzzer s’arrête de sonner
}
delay(100);

  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù

//*****&&&&&&&&&&&&*********++++MODE 19 ++++*******&&&&&&&&&&&*********

//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù

//*****&&&&&&&&&&&&*********++++MODE 20 DHT11 SHOW TEM DATA LCD++++*******&&&&&&&&&&&*********
void DHT11_MODE_20(){
  DHT.read11(dht_apin);
  //FLAGS DHT.temperature & DHT.humidity
  }
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù

//*****&&&&&&&&&&&&*********++++MODE 21 ++++*******&&&&&&&&&&&*********
int sensorThreshold = 400;
void Gas_SENSOR_MODE21(){
    int analogSensor = analogRead(A12);

   if (analogSensor > sensorThreshold)
  {
    digitalWrite(LED2, HIGH);
    digitalWrite(LED1, LOW);
    tone(buzzerPin, 1000, 200);
  }
  else
  {
    digitalWrite(LED2, LOW);
    digitalWrite(LED1, HIGH);
    noTone(buzzerPin);
  }
  delay(100);
}
  void photoresist() {

  valeur = analogRead(A7);
  //F INFO valeur
  delay(250);
}
//ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù



/***************************TEST*************////////////////*********FUNCTIONS*********************/*/

void led_blink_fast(){
  digitalWrite(LED, 1);
  delay(500);
  digitalWrite(LED, 0);
  delay(500);
  func1_val = 12;
}

void led_blink_slow(){
  digitalWrite(LED, 1);
  delay(1000);
  digitalWrite(LED, 0);
  delay(1000);
  func1_val2 = 10;
}

void DHTSENSOR(){
  DHT.read11(dht_apin);
  dht_val = (int)DHT.humidity;
}
/*************************************************************************/






void setup(){
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);



  /*DECLARATIONS*/
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  noTone(buzzerPin);
  myStepper.setSpeed(100);
   //pinMode(SW_pin, INPUT); // Switch pin is an input
//  digitalWrite(SW_pin, HIGH); //set to high
  pinMode(redPin, OUTPUT); // declares RGB LED as an output
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(A7, INPUT);
  pinMode(potpiin, INPUT);

  myservo.attach(35);

}


void loop() {
  if (flag == 1){
    led_blink_fast();
  }else if(flag==2){
    DHTSENSOR();
  }else if(flag==3){
    ledblink_MODE1();
  }else if(flag==4){
    ledbuttonControl_MODE3();
  }else if(flag==5){
    lcdPrintButState_MODE2();
  }else if(flag==6){
    PWM_Led_MODE_6();
  }else if(flag==7){
    dc_motor_speed_MODE_7();
  }else if(flag==8){
    servopotlcd_MODE_8();
  }else if(flag==9){
    stepbystep_MODE_9();
  }else if(flag==10){
    Knob_STEPPER_MODE10();
  }else if(flag==11){
    JoyServoControl_MODE_11();
  }else if(flag==12){
    HC05_LEDonoff_MODE_13();
  }else if(flag==13){
    ultrason_lcd_MODE_14();
  }else if(flag==14){
    infra_obst_detec_MODE_15();
  }else if(flag==15){
    PIR_MODE_16();
  }else if(flag==16){
    CAP_Son_MODE_17();
  }else if(flag==17){
    flamme_detec_MODE_18();
  }else if(flag==18){
    DHT11_MODE_20();
  }else if(flag==19){
    Gas_SENSOR_MODE21();
  }
    delay(10);

  // Everything happens in the interrupts
}
