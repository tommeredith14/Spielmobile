 /*
Created by Tom Meredith on 2016-09-09.
Copyright (c) 2016 Geoff Spielman. All rights reserved.
*/

#define DEBUG //Comment line for real use


//Ecoder pin defines
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 4
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 5

//global encoder count 
//must be volatile for safe reading/writing in interupt/main program
volatile int LeftCounts = 0;
volatile int RightCounts = 0;


String recString = "";

int enablePin = 11;
int in1Pin = 10;
int in2Pin = 9;
int switchPin = 7;
int potPin = 0;

void setup() {
  Serial.begin(38400);
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), LeftEncISR, FALLING);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightEncISR, FALLING);
  recString.reserve(400);

    pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
}
bool reverse = false;
void loop() {

    int speed = 200;
  //boolean reverse = digitalRead(switchPin);
  
  setMotor(speed, reverse);
  while (LeftCounts < 2100){}
  reverse = !reverse;
  LeftCounts = 0;
}//end loop


//expect morse code to start with . or - and sequences are in the form 12031021, BOTH END WITH ~
void serialEvent()
{
  while (Serial.available())
  {  
    char inChar = (char)Serial.read();
    recString += inChar;
    
    if (inChar == '~')
    {
      
       
      
     
    }
  }
}
void setMotor(int speed, boolean reverse)
{
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, ! reverse);
  digitalWrite(in2Pin, reverse);
}
//////Interupt handlers///////
void LeftEncISR(){
  bool Bvalue = 1;//digitalRead(LEFT_ENCODER_B);
  if (Bvalue)
  {
    LeftCounts++;
    
  }
  #ifdef DEBUG
  //Serial.println("Interupted");
  //Serial.print("LEFT: ");
  //if (LeftCounts %1 == 0)Serial.println(LeftCounts);
  #endif
}
void RightEncISR(){
  bool Bvalue = digitalRead(RIGHT_ENCODER_B);
  if (Bvalue)
  {
    RightCounts++;
    
  }
  #ifdef DEBUG
  Serial.println("Interupted");
  Serial.print("Right: ");
  Serial.println(RightCounts);
  #endif
}
//Interupt handlers complete//


void resetButtonLog(){

}
