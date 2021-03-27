/*
* Copyright (c) 2015, Xiaoer Geek Technology Co., Ltd.
* All rights reserved.
*
* File name: wifi-robots
* File identification:
* Abstract: Wi-Fi robot smart car control
*
* Current version: 2560 V2.3
* Author: Little R Team
* Date of completion: April 1, 2015
*/
#include <Servo.h>
#include <EEPROM.h>
#include <LCD12864RSPI.h>
#define SIZE( a) sizeof( a) / sizeof( a[0])

/************************************************* ********************12864 LCD screen ***************************** *******************************************/
/*
LCD Arduino
PIN1 = GND
PIN2 = 5V
RS(CS) = 8;
RW(SID) = 9;
EN(CLK) = 3;
PIN15 PSB = GND;
*/

#define INIT 0
#define NORMAL 0
#define FOLLOW 1
#define AVOID 2
#define WAVEAVOID 3
static int Level = 0;
static int Mode = 0;
int Refresh = 0;
unsigned char show0[] = {0xD0, 0xA1, 0xB6, 0xFE, 0xBF, 0xC6, 0xBC, 0xBC}; //Xiaoer Technology
unsigned char show1[] = "wifi-robots";
unsigned char Normal_S[] = {
  0xA1, 0xF1,
  0xD5, 0xFD,
  0xB3, 0xA3,
  0xC4, 0xA3,
  0xCA, 0xBD
}; //Normal mode
unsigned char Normal[]={
  0xD5, 0xFD,
  0xB3, 0xA3,
  0xC4, 0xA3,
  0xCA, 0xBD
     }; //Normal mode
unsigned char Follow_S[] = {
  0xA1, 0xF1,
  0xBA, 0xEC,
  0xCD, 0xE2,
  0xD1, 0xAD,
  0xBC, 0xA3
}; //Infrared tracking
unsigned char follow[] = {
  0xBA, 0xEC,
  0xCD, 0xE2,
  0xD1, 0xAD,
  0xBC, 0xA3
}; //Infrared tracking
unsigned char Avoid_S[]={
  0xA1, 0xF1,
  0xBA, 0xEC,
  0xCD, 0xE2,
  0xB1, 0xDC,
  0xD5, 0xCF
}; //Infrared obstacle avoidance
unsigned char Avoid[]={
  0xBA, 0xEC,
  0xCD, 0xE2,
  0xB1, 0xDC,
  0xD5, 0xCF
}; //Infrared obstacle avoidance
unsigned char WaveAvoid_S[] = {
  0xA1, 0xF1,
  0xB3, 0xAC,
  0xC9, 0xF9,
  0xB2, 0xA8,
  0xB1, 0xDA,
  0xD5, 0xCF
}; //Ultrasonic barrier
unsigned char WaveAvoid[] = {
  0xB3, 0xAC,
  0xC9, 0xF9,
  0xB2, 0xA8,
  0xB1, 0xDA,
  0xD5, 0xCF
}; //Ultrasonic barrier

/************************************************* ********************12864 LCD screen ***************************** *******************************************/

int ledpin1 = A6;//Set the system startup indicator 1
int ledpin2 = A7;//Set the system startup indicator 2
int ENA = 5;//L298 enable A
int ENB = 6;//L298 enable B
int INPUT2 = 7;//Motor interface 1
int INPUT1 = 8;//Motor interface 2
int INPUT3 = 12;//Motor interface 3
int INPUT4 = 13;//Motor interface 4
int Key1_times;
int Key2_times;
boolean MoterStatusLED = true;
boolean ServoStatusLED = true;

int adjust = 1;//Define the motor flag
int Echo = A5; // Define the ultrasonic signal receiving pin
int Trig = A4; // Define the ultrasound signal transmitter pin
int Input_Detect_LEFT = A14; //Define the left infrared of the car to track
int Input_Detect_RIGHT = A13; //Define the infrared on the right side of the car's tracking
int Input_Detect = A1;//Define the infrared in front of the car
int Input_Detect_TrackLeft = A3;//Define the car to follow the left infrared
int Input_Detect_TrackRight = A2;//Define the car to follow the infrared on the right
int Carled = A0;//Define the car light interface
int Cruising_Flag = 0;
int Pre_Cruising_Flag = 0;
int Left_Speed_Hold = 255;//Define the left speed variable
int Right_Speed_Hold = 255;//Define the right speed variable

Servo servo1;// Create servo #1
Servo servo2;// Create servo #2
Servo servo3;// Create a servo#3
Servo servo4; // Create a servo #4
Servo servo5; // Create a servo #5
Servo servo6; // Create a servo #6
Servo servo7; // Create a servo #7
Servo servo8; // Create a servo #8

byte angle1 = 70; //Initial value of servo #1
byte angle2 = 60; //Initial value of servo #2
byte angle3 = 60; //Servo#3 initial value
byte angle4 = 60; //Servo#4 initial value
byte angle5 = 60; //Initial value of servo #5
byte angle6 = 60; //Servo#6 initial value
byte angle7 = 60; //Servo#7 initial value
byte angle8 = 60; //Servo#8 initial value

int buffer[3]; //Serial port receiving data buffer
int rec_flag; //Serial port receiving flag
int serial_data;
int Uartcount;
int IR_R;
int IR_L;
int IR_TL;
int IR;
int IR_TR;
unsigned long Pretime;
unsigned long Nowtime;
unsigned long Costtime;

#define MOTOR_GO_FORWARD {digitalWrite(INPUT1,LOW);digitalWrite(INPUT2,HIGH);digitalWrite(INPUT3,LOW);digitalWrite(INPUT4,HIGH);} //Car body forward
#define MOTOR_GO_BACK {digitalWrite(INPUT1,HIGH);digitalWrite(INPUT2,LOW);digitalWrite(INPUT3,HIGH);digitalWrite(INPUT4,LOW);} //car body back
#define MOTOR_GO_RIGHT {digitalWrite(INPUT1,HIGH);digitalWrite(INPUT2,LOW);digitalWrite(INPUT3,LOW);digitalWrite(INPUT4,HIGH);} //Car body turns right
#define MOTOR_GO_LEFT {digitalWrite(INPUT1,LOW);digitalWrite(INPUT2,HIGH);digitalWrite(INPUT3,HIGH);digitalWrite(INPUT4,LOW);} //Car body turns left
#define MOTOR_GO_STOP {digitalWrite(INPUT1,LOW);digitalWrite(INPUT2,LOW);digitalWrite(INPUT3,LOW);digitalWrite(INPUT4,LOW);} //The car body stops

/*
Calibrate the direction of the trolley by the calibration value
*/
void forward(int adjust)
{
  switch (adjust)
  {
    case 1: MOTOR_GO_FORWARD; return;
    case 2: MOTOR_GO_FORWARD; return;
    case 3: MOTOR_GO_BACK; return;
    case 4: MOTOR_GO_BACK; return;
    case 5: MOTOR_GO_LEFT; return;
    case 6: MOTOR_GO_LEFT; return;
    case 7: MOTOR_GO_RIGHT; return;
    case 8: MOTOR_GO_RIGHT; return;
    default: return;
  }
}
/*
Calibrate the direction of the trolley by the calibration value
*/
void back(int adjust)
{
  switch (adjust)
  {
    case 1: MOTOR_GO_BACK; return;
    case 2: MOTOR_GO_BACK; return;
    case 3: MOTOR_GO_FORWARD; return;
    case 4: MOTOR_GO_FORWARD; return;
    case 5: MOTOR_GO_RIGHT; return;
    case 6: MOTOR_GO_RIGHT; return;
    case 7: MOTOR_GO_LEFT; return;
    case 8: MOTOR_GO_LEFT; return;
    default: return;
  }
}
/*
Calibrate the direction of the car by the calibration value
*/
void left(int adjust)
{
  switch (adjust)
  {
    case 1: MOTOR_GO_LEFT; return;
    case 2: MOTOR_GO_RIGHT; return;
    case 3: MOTOR_GO_LEFT; return;
    case 4: MOTOR_GO_RIGHT; return;
    case 5: MOTOR_GO_FORWARD; return;
    case 6: MOTOR_GO_BACK; return;
    case 7: MOTOR_GO_FORWARD; return;
    case 8: MOTOR_GO_BACK; return;
    default: return;
  }
}
/*
Calibrate the direction of the car by the calibration value
*/
void right(int adjust)
{
  switch (adjust)
  {
    case 1: MOTOR_GO_RIGHT; return;
    case 2: MOTOR_GO_LEFT; return;
    case 3: MOTOR_GO_RIGHT; return;
    case 4: MOTOR_GO_LEFT; return;
    case 5: MOTOR_GO_BACK; return;
    case 6: MOTOR_GO_FORWARD; return;
    case 7: MOTOR_GO_BACK; return;
    case 8: MOTOR_GO_FORWARD; return;
    default: return;
  }
}

/*
************************************************** ************************************************** *****
** Function name: Open_Light
** Function function: turn on the car lights
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Open_Light()//Turn on the headlights
{
  digitalWrite(Carled, HIGH); //Pull the low level, connect the positive pole to the power supply, and connect the negative pole to the Io port
  delay(1000);
}

 /*
************************************************** ************************************************** *****
** Function name: Close_Light
** Function: Turn off the lights
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Close_Light()//Turn off the headlights
{
  digitalWrite(Carled, LOW); //Pull the low level, connect the positive pole to the power supply, and connect the negative pole to the Io port
  delay(1000);
}

/*
************************************************** ************************************************** *****
** Function name: Avoiding
** Function function: detect obstacles in front of the infrared in the middle of the front of the car body, if there is, the car will stop
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Avoiding()//Infrared obstacle avoidance function
{
  IR = digitalRead(Input_Detect);
  if ((IR == HIGH))
  {
    forward(adjust);//Go straight
    return;
  }
  if ((IR == LOW))
  {
    MOTOR_GO_STOP;//Stop
    return;
  }
}
    
/*
************************************************** ************************************************** *****
** Function name: Follow
**Function function: detect the position of the black line between the two infrared rays, and then make the direction change of the trolley through logical judgment
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Follow() //Follow mode
{
    IR = digitalRead(Input_Detect);
    IR_TL = digitalRead(Input_Detect_TrackLeft);
    IR_TR = digitalRead(Input_Detect_TrackRight);
    if(IR == 1) //Intermediate sensor OK
{
      if((IR_TL == 0)&& (IR_TR == 0)) // Obstacles are detected on both sides at the same time
      {
      MOTOR_GO_STOP;//Stop
      }
      if((IR_TL == 0)&& (IR_TR == 1))//obstacles on the left
      {
      right(adjust); //Turn right
      }
      if((IR_TL == 1)&& (IR_TR == 0))//obstacles on the right
      {
      left(adjust); //Turn left
      }
      if((IR_TL == 1)&& (IR_TR == 1))//No obstacles
      {
            forward(adjust);//Go straight
      }
    }
else
{
MOTOR_GO_STOP;
}
}

/*
************************************************** ************************************************** *****
** Function name: TrackLine
**Function function: detect the position of the black line between the two infrared rays, and then make the direction change of the trolley through logical judgment
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void TrackLine() // Line tracking mode
{
  IR_L = digitalRead(Input_Detect_LEFT);//Read the left sensor value
  IR_R = digitalRead(Input_Detect_RIGHT);//Read the value of the right sensor

  if ((IR_L == LOW) && (IR_R == LOW)) // Obstacles are detected at the same time on both sides
  {
    forward(adjust);//Go straight
    return;

  }
  if ((IR_L == LOW) && (IR_R == HIGH)) //I encounter obstacles on the right
  {
    left(adjust);//Turn left
    return;

  }
  if ((IR_L == HIGH) && (IR_R == LOW)) //Encountered obstacles on the left
  {
    right(adjust);//Turn right
    return;

  }
  if ((IR_L == HIGH) && (IR_R == HIGH)) // Both left and right are detected, just like a horizontal tape in the video
  {
    MOTOR_GO_STOP;//Stop
    return;
  }
}

/*
************************************************** ************************************************** *****
** Function name: Get_Distence
** Function: Detect and return the measured distance value of ultrasonic wave (unit cm)
** Entry parameters: none
** Export parameters: Ldistance
************************************************** ************************************************** *****
*/
char Get_Distance()//Measure the distance
{
  digitalWrite(Trig, LOW); // Let the ultrasound transmit low voltage 2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH); // Let the ultrasound transmit high voltage for 10μs, here it is at least 10μs
  delayMicroseconds(10);
  digitalWrite(Trig, LOW); // maintain low voltage of ultrasonic emission
  float Ldistance = pulseIn(Echo, HIGH); // Read the difference time
  Ldistance = Ldistance / 5.8 / 10; // Convert time to distance distance (unit: cm)
  //Serial.println(Ldistance); //Display distance
  return Ldistance;
}

/*
************************************************** ************************************************** *****
** Function name: AvoidByRadar
** Function: When the distance detected by the ultrasonic is less than distance (in cm), the car will stop
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void AvoidByRadar(int distance)//Ultrasonic obstacle avoidance function
{
  if(Get_Distance() <distance)//obstacle avoidance distance value (unit cm)
    {
        MOTOR_GO_STOP;
    }
    else
    {
        forward(adjust);
    }
}


/*
************************************************** ************************************************** *****
** Function name: Send_Distance
** Function: Send ultrasonic data to the host computer (data format: 0XFF, 0X03, angle (default 0X00), distance (dis), 0XFF)
** Entry parameters: none
** Export parameters: none
************************************************** *************************************************** ****
*/
void Send_Distance()//Ultrasonic distance display on PC
{
  int dis = Get_Distance();
  Serial.write(0xff);
  Serial.write(0x03);
  Serial.write(0x00);
  Serial.write(dis);
  Serial.write(0xff);
  delay(1000);
}
/*
************************************************** ************************************************** *****
** Function name: Delayed()
** Function function: Delay program
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Delayed() //Delay for 40 seconds to wait for the WIFI module to start
{
  int i;
  for (i = 0; i <28; i++)
  {
    digitalWrite(ledpin1, HIGH);
    digitalWrite(ledpin2, LOW);
    delay(1000);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, HIGH);
    delay(1000);
  }
  
  for (i = 0; i <10; i++)
  {
    digitalWrite(ledpin1, HIGH);
    digitalWrite(ledpin2, HIGH);
    delay(500);
    digitalWrite(ledpin1, LOW);
    digitalWrite(ledpin2, LOW);
    delay(500);
  }
  MOTOR_GO_STOP;
  digitalWrite(ledpin1, LOW);
  digitalWrite(ledpin2, LOW);
}



/************************************************* ************************************************** ************************************************** ******************************************/
                                                                /*12864 LCD related functions*/


/************************************************* ************
                        External interrupt 2 function Key1()
************************************************** ***********/
void Key1()
{
 if(Key1_times==0)
{
  Key1_times=7000;
  MENU(Level,Mode);MENU(Level,Mode);
  Refresh = 1;
  switch(Level)
  {
    case 0:Level = 1; Mode = 0; MENU(Level,Mode);MENU(Level,Mode);return;
    case 1:Level = 2;
           switch(Mode)
           {
             case NORMAL:Cruising_Flag = 0; MENU(Level,Mode);MENU(Level,Mode);return;
             case FOLLOW:Cruising_Flag = 2; MENU(Level,Mode);MENU(Level,Mode);return;
             case AVOID: Cruising_Flag = 3; MENU(Level,Mode);MENU(Level,Mode);return;
             case WAVEAVOID:Cruising_Flag =4; MENU(Level,Mode);MENU(Level,Mode);return;
             default:Cruising_Flag = 0; MENU(Level,Mode);MENU(Level,Mode);return;
           }
             MENU(Level,Mode);MENU(Level,Mode);return;
    default:Level = 2; MENU(Level,Mode);MENU(Level,Mode);return;
  }
 MENU(Level,Mode);MENU(Level,Mode);
 }
}
/************************************************* ************
                        External interrupt 3 function Key2()
************************************************** ***********/
void Key2()
{
if(Key2_times==0)
{
  Key2_times=7000;
  MENU(Level,Mode);MENU(Level,Mode);
  Refresh = 1;
  switch(Level)
  {
    case 0: MENU(Level,Mode);MENU(Level,Mode);return;
    case 1:Mode++;if (Mode> 3)Mode = 0; MENU(Level,Mode);MENU(Level,Mode);return;
    default:Level = 1;Cruising_Flag = 0; MENU(Level,Mode);MENU(Level,Mode);return;
  }
 MENU(Level,Mode);MENU(Level,Mode);
}
}
/************************************************* ************
                       Menu option display
************************************************** ***********/

void MENU(int Level,int Mode)
{
  
  if(Refresh)
  {
    Refresh = 0;
    LCDA.CLEAR();
  }
  delay(10);//This delay must be
  switch(Level)
  {
    case 0:
         if(!Mode)
         {
           LCDA.DisplayString(0, 2, show0, SIZE(show0)); //Start from the third grid on the first line, display text Xiaoer Technology
           LCDA.DisplayString(2, 1, show1, SIZE(show1)); //Start from the second grid on the third line, display text wifi-robots
         }
         return;
    case 1:
          switch(Mode)
          {
            case NORMAL: LCDA.DisplayString(0, 0, Normal_S, SIZE(Normal_S));
                           LCDA.DisplayString(1, 1, follow, SIZE(follow));
                           LCDA.DisplayString(2, 1, Avoid, SIZE(Avoid));
                           LCDA.DisplayString(3, 1, WaveAvoid, SIZE(WaveAvoid));
                           return; //Selected item is normal mode
            case FOLLOW: LCDA.DisplayString(0, 1, Normal, SIZE(Normal));
                           LCDA.DisplayString(1, 0, Follow_S, SIZE(Follow_S));
                           LCDA.DisplayString(2, 1, Avoid, SIZE(Avoid));
                           LCDA.DisplayString(3, 1, WaveAvoid, SIZE(WaveAvoid));
                           return; //The option is infrared tracking mode
            case AVOID: LCDA.DisplayString(0, 1, Normal, SIZE(Normal));
                           LCDA.DisplayString(1, 1, follow, SIZE(follow));
                           LCDA.DisplayString(2, 0, Avoid_S, SIZE(Avoid_S));
                           LCDA.DisplayString(3, 1, WaveAvoid, SIZE(WaveAvoid));
                           return; //The option is infrared barrier mode
            case WAVEAVOID:LCDA.DisplayString(0, 1, Normal, SIZE(Normal));
                           LCDA.DisplayString(1, 1, follow, SIZE(follow));
                           LCDA.DisplayString(2, 1, Avoid, SIZE(Avoid));
						   LCDA.DisplayString(3, 0, WaveAvoid_S, SIZE(WaveAvoid_S));
                           return; //The option is ultrasonic barrier mode
          }
          return;
    case 2:
          switch(Mode)
          {
            case NORMAL:LCDA.DisplayString(1, 2, Normal, SIZE(Normal));return; //Normal mode
            case FOLLOW:LCDA.DisplayString(1, 2, follow, SIZE(follow));return; //Infrared tracking mode
            case AVOID: LCDA.DisplayString(1, 2, Avoid, SIZE(Avoid)); return; //Infrared barrier mode
            case WAVEAVOID:LCDA.DisplayString(1, 1, WaveAvoid, SIZE(WaveAvoid));return; //Ultrasonic barrier mode
          }
          return;
    default:return;
  }

}
                                                                    /*12864 LCD related functions */
/************************************************* ************************************************** ************************************************** ******************************************/


/*
************************************************** ************************************************** *****
** Function name: setup().Init_Steer()
** Function function: system initialization (serial port, motor, servo, indicator initialization).
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Init_Steer()//Initialize the servo (the angle is the last saved value)
{
  angle1 = EEPROM.read(0x01);//Read the value in register 0x01
  angle2 = EEPROM.read(0x02);//Read the value in register 0x02
  angle3 = EEPROM.read(0x03);//Read the value in register 0x03
  angle4 = EEPROM.read(0x04);//Read the value in register 0x04
  angle5 = EEPROM.read(0x05);//Read the value in register 0x05
  angle6 = EEPROM.read(0x06);//Read the value in register 0x06
  angle7 = EEPROM.read(0x07);//Read the value in register 0x07
  angle8 = EEPROM.read(0x08);//Read the value in register 0x08
  if ((angle1 == 255)|| (angle2 == 255)||( angle2 == 255)|| (angle2 == 255)||( angle2 == 255)|| (angle2 == 255)|| (angle2 == 255)|| (angle2 == 255)|| (angle2 == 255))
  {
    EEPROM.write(0x01, 60); //store the initial angle in address 0x01
    EEPROM.write(0x02, 60); //store the initial angle in address 0x02
    EEPROM.write(0x03, 60); //store the initial angle in address 0x03
    EEPROM.write(0x04, 60); //store the initial angle in address 0x04
    EEPROM.write(0x05, 60); //store the initial angle in address 0x05
    EEPROM.write(0x06, 60); //store the initial angle in address 0x06
    EEPROM.write(0x07, 60); //store the initial angle in address 0x07
    EEPROM.write(0x08, 60); //store the initial angle in address 0x08
    return;
  }
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  servo4.write(angle4);
  servo5.write(angle5);
  servo6.write(angle6);
  servo7.write(angle7);
  servo8.write(angle8);
  adjust = EEPROM.read(0x10);//Read the value in register 0x10
  if (adjust == 0xff)EEPROM.write(0x10, 1);
  
    Left_Speed_Hold = EEPROM.read(0x09);//Read the value in register 0x03
    Right_Speed_Hold = EEPROM.read(0x0A);//Read the value in register 0x04
    if((Left_Speed_Hold<55)||(Right_Speed_Hold<55))
    {
       Left_Speed_Hold=255;
       Right_Speed_Hold=255;
     }
    analogWrite(ENB,Left_Speed_Hold);//Assign a value to L298 enable terminal B
    analogWrite(ENA,Right_Speed_Hold);//Assign a value to L298 enable terminal A
}
void setup()
{
  pinMode(ledpin1, OUTPUT);
  pinMode(ledpin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(INPUT3, OUTPUT);
  pinMode(INPUT4, OUTPUT);
  pinMode(Input_Detect_LEFT, INPUT);
  pinMode(Input_Detect_RIGHT, INPUT);
  pinMode(Input_Detect_TrackLeft,INPUT);
  pinMode(Input_Detect_TrackRight,INPUT);
  pinMode(Carled, OUTPUT);
  pinMode(Input_Detect, INPUT);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  digitalWrite(20, HIGH);
  digitalWrite(21, HIGH);
  LCDA.Initialise(); // screen initialization
  LCDA.CLEAR();//Clear the screen
  MENU(INIT,INIT);
  Delayed();//Delay for 50 seconds to wait for the WIFI module to start up
  attachInterrupt(2, Key1, FALLING);
  attachInterrupt(3, Key2, FALLING);

  servo1.attach(11);//Define the servo 7 control port
  servo2.attach(2);//Define the servo 8 control port
  servo3.attach(4);//Define the servo 7 control port
  servo4.attach(3);//Define the servo 8 control port
  servo5.attach(A8);//define servo 7 control port
  servo6.attach(A9);//define servo 8 control port
  servo7.attach(9);//Define the servo 7 control port
  servo8.attach(10);//Define the servo 8 control port

  Serial.begin(9600);//Set the serial port baud rate to 9600 bps
  Init_Steer();
}
/*
************************************************** ************************************************** *****
** Function name: loop()
** Function function: main function
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Cruising_Mod()//Mode function switching function
{

  if (Pre_Cruising_Flag != Cruising_Flag)
  {
    if (Pre_Cruising_Flag != 0)
    {
      MOTOR_GO_STOP;
    }

    Pre_Cruising_Flag = Cruising_Flag;
  }
  switch (Cruising_Flag)
  {
    case 1: Follow();//Follow mode
    case 2: TrackLine(); return; //Line tracking mode
    case 3: Avoiding(); return; //Obstacle avoidance mode
    case 4: AvoidByRadar(15); return; //Ultrasonic obstacle avoidance mode
    case 5: Send_Distance(); //Ultrasonic distance display on PC
    default: return;
  }

}

void loop()
{

  MENU(Level,Mode);
  while (1)
  {
    Get_uartdata();
    UartTimeoutCheck();
    Cruising_Mod();
    if(Key1_times>0){Key1_times--;}
    if(Key2_times>0){Key2_times--;}
  }
}



/*
************************************************** ************************************************** *****
** LetterNumber name: Communication_Decode()
**Function function: serial port command decoding
** Entry parameters: none
** Export parameters: none
* * Left speed : buffer[0][1][2]
** 0x0002 Int value:2
** 0x0001 Int value:1
** 0x004F Int value:79
* Right speed
** 0x0002 Int value:2
** 0x0001 Int value:2
** 0x004F Int value:79
************************************************** ************************************************** *****
*/
void Communication_Decode()
{
    char tmp[16];
  
  Serial.print("In the decode\n");

   for (int i=0; i<3; i++)
       {
         sprintf(tmp, "0x%.4X",buffer[i]);
         Serial.print(tmp); Serial.print(" ");
         Serial.print("Int value:");
         Serial.print (buffer[i]);
         Serial.print("\n");
       }


  Serial.print("\n");
  if (buffer[0] == 0x00)
  {
    MoterStatusLED=!MoterStatusLED;
    digitalWrite(ledpin1,MoterStatusLED);
    switch (buffer[1]) //Motor command
    {
      case 0x01: MOTOR_GO_FORWARD; return;
      case 0x02: MOTOR_GO_BACK; return;
      case 0x03: MOTOR_GO_LEFT; return;
      case 0x04: MOTOR_GO_RIGHT; return;
      case 0x00: MOTOR_GO_STOP; return;
    }
  }
  else if (buffer[0] == 0x01) //Servo command
  {
    ServoStatusLED=!ServoStatusLED;
    digitalWrite(ledpin2,ServoStatusLED);
    if (buffer[2]> 170)return;
    switch (buffer[1])
    {
      case 0x01: angle1 = buffer[2]; servo1.write(angle1); return;
      case 0x02: angle2 = buffer[2]; servo2.write(angle2); return;
      case 0x03: angle3 = buffer[2]; servo3.write(angle3); return;
      case 0x04: angle4 = buffer[2]; servo4.write(angle4); return;
      case 0x05: angle5 = buffer[2]; servo5.write(angle5); return;
      case 0x06: angle6 = buffer[2]; servo6.write(angle6); return;
      case 0x07: angle7 = buffer[2]; servo7.write(angle7); return;
      case 0x08: angle8 = buffer[2]; servo8.write(angle8); return;
      default: return;
    }

  }

  else if (buffer[0] == 0x02) //speed adjustment
  {
    if (buffer[2]> 100)return;

    if (buffer[1] == 0x01) //shift on the left
    {
      Left_Speed_Hold=buffer[2]*2+55;//The speed gear is 0~100, converted to pwm, the speed pwm is lower than 55, the motor does not rotate
      analogWrite(ENB, Left_Speed_Hold);
      EEPROM.write(0x09,Left_Speed_Hold);//Storage speed
    }
    if (buffer[1] == 0x02) //shift on the right
    {
      Right_Speed_Hold=buffer[2]*2+55;//The speed gear is 0~100, converted to pwm, the speed pwm is lower than 55, the motor does not rotate
      analogWrite(ENA,Right_Speed_Hold);
      EEPROM.write(0x0A,Right_Speed_Hold);//Storage speed
    } else return;
  }
  else if (buffer[0] == 0x33) //Read the servo angle and assign it
  {
    Init_Steer(); return;
  }
  else if (buffer[0] == 0x32) //Save the command
  {
    EEPROM.write(0x01, angle1);
    EEPROM.write(0x02, angle2);
    EEPROM.write(0x03, angle3);
    EEPROM.write(0x04, angle4);
    EEPROM.write(0x05, angle5);
    EEPROM.write(0x06, angle6);
    EEPROM.write(0x07, angle7);
    EEPROM.write(0x08, angle8);
    return;
  }
  else if (buffer[0] == 0x13) //Mode switch
  {
    switch (buffer[1])
    {
      case 0x01: Cruising_Flag = 1; return;//Follow
      case 0x02: Cruising_Flag = 2;Level = 2;Mode = 1;Refresh = 1; return;//Line patrol
      case 0x03: Cruising_Flag = 3;Level = 2;Mode = 2;Refresh = 1; return;//obstacle avoidance
      case 0x04: Cruising_Flag = 4;Level = 2;Mode = 3;Refresh = 1; return;//Radar obstacle avoidance
      case 0x05: Cruising_Flag = 5; return;//Ultrasonic distance display on PC
      case 0x00: Cruising_Flag = 0;Level = 2;Mode = 0;Refresh = 1; return;//Normal mode
      default: Cruising_Flag = 0;Level = 2;Mode = 0;Refresh = 1; return; //Normal mode
    }
  }
  else if (buffer[0] == 0x04)//The command for driving lights is FF040000FF, and the command for turning off lights is FF040100FF
  {
    switch (buffer[1])
    {
      case 0x00: Open_Light(); return; //driving light
      case 0x01: Close_Light(); return; //Turn off the lights
      default: return;
    }
  }
  else if (buffer[0] == 0x40) //store motor flag
  {
    adjust = buffer[1];
    EEPROM.write(0x10, adjust);
  }


}
/*
************************************************** ************************************************** *****
** Function name: Get_uartdata()
** Function: read serial port commands
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void Get_uartdata(void)
{
  static int i;

  if (Serial.available()> 0) //Determine whether the serial port buffer has data loaded
  {
    serial_data = Serial.read();//Read the serial port
    if (rec_flag == 0)
    {
      if (serial_data == 0xff)//Get 0xff for the first time (i.e. packet header)
      {
        rec_flag = 1;
        i = 0;
        Costtime = 0;
      }
    }
    else
    {
      if (serial_data == 0xff)//Get 0xff for the second time (that is, the end of the packet)
      {
        rec_flag = 0;
        if (i == 3)//The intermediate data obtained is 3 bytes, indicating that the command format is correct
        {
          Communication_Decode();//Execute command analysis function
        }
        i = 0;
      }
      else
      {
        buffer[i] = serial_data;//temporary data
        i++;
      }
    }
  }
}
/*
************************************************** ************************************************** *****
** Function name: UartTimeoutCheck()
**Function function: serial port timeout detection
** Entry parameters: none
** Export parameters: none
************************************************** ************************************************** *****
*/
void UartTimeoutCheck(void)
{
  if (rec_flag == 1)
  {
    Costtime++;
    if (Costtime == 100000)
    {
      rec_flag = 0;
    }
  }
}
