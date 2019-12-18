

#include <TimerOne.h>
// left
#define ENA 3
#define INa1 4
#define INa2 5
//right
#define INb1 7
#define INb2 6
#define ENB 11
//
int sensor0 = A0;      // Left most sensor
int sensor1 = A1;
int sensor2 = A2;
int sensor3 = A3;      // Right most sensor
int sensor4 = A4;



unsigned int counter1 = 0;
unsigned int counter2 = 0;


#define rightMaxSpeed 66
#define leftMaxSpeed 66   

//PID
float Kp=5,Ki=0.000,Kd=50;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
float tests = 0;
// velocity
int initial_motor_speed =80;

int Stop = 0;
int count=0;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);


int LM, RM = 0;


void docount1()  // counts from the speed sensor
{
  counter1++;

}
void docount2()  // counts from the speed sensor
{

  counter2++;// increase +1 the counter value
}


void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  Serial.print(" LEFT Motor Speed: ");
  int rotation = (counter2 / 24);  // divide by number of holes in Disc
  Serial.print(rotation, DEC);
  Serial.println(" Rotation per seconds");
  Serial.print("RIGHT Motor Speed: ");
  int rotation1 = (counter1 / 24);  // divide by number of holes in Disc
  Serial.print(rotation1, DEC);
  Serial.println(" Rotation per seconds");

//to monitor the status of IR-sensors and pwm generated for each motor//////


  Serial.print("LEFT SENSOR = ");
  Serial.println(sensor[0]);
  Serial.print("Sensor 4 = ");
  Serial.println(sensor[1]);

  Serial.print("MIDDLE SENSOR = ");
  Serial.println(sensor[2]);
 
  Serial.print("Sensor 2 = ");
  Serial.println(sensor[3]);

  Serial.print("RIGHT SENSOR = ");
  Serial.println(sensor[4]);
  Serial.println("-------------");

  Serial.print("Left motor PWM = ");
  Serial.println(LM);

  Serial.print("Right motor PWM = ");

  Serial.println(RM);


  




  Serial.println("");
  Serial.println("");


  counter1 = 0;
  counter2 = 0; //  reset counter to zero
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}


void setup()
{
  pinMode(sensor0, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INa1,OUTPUT);
  pinMode(INa2,OUTPUT);
  pinMode(INb1,OUTPUT);
  pinMode(INb2,OUTPUT);
  Serial.begin(9600); 
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(0, docount2, RISING);
  attachInterrupt(1, docount1, RISING); // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer


  Serial.begin(9600); //Enable Serial Communications

}

void loop()
{ 
  //Serial.print("error=");
  //Serial.println(error);
  //delay(1000);
  read_sensor_values();

  calculate_pid();

  motor_control();
 




}

void read_sensor_values()
{
  sensor[0] = digitalRead(A0);  //left
  sensor[1] = digitalRead(A1);
  sensor[2] = digitalRead(A2);   //middle
  sensor[3] = digitalRead(A3);
  sensor[4] = digitalRead(A4);    //right
  



// turn_right
 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = -6.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = -5.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    error = -4.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = -2;
// on the middle
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
  {
    error = 0;
  }
  // turn left
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 4.5;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 5.5;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 6.5;
//
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))

    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))

    error = 3;

  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
    error = -3;
   else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
    error = -3;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 0;
 }
void calculate_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  //previous_I = error;
  previous_error = error; 
}

void motor_control()
{

  // Calculating the effective motor speed:
  int leftMotorSpeed = initial_motor_speed + PID_value;
  int rightMotorSpeed  = initial_motor_speed - PID_value;



  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  if (rightMotorSpeed < 45) rightMotorSpeed = 0; // defining lower limits of the pwm generated.
  if (leftMotorSpeed < 45) leftMotorSpeed = 0; // defining lower limits of the pwm generated.

  //following lines of code are to make the bot move forward

  analogWrite(ENA, leftMotorSpeed); //left Motor Speed
  analogWrite(ENB, rightMotorSpeed); //right Motor speed
  LM = leftMotorSpeed;
  RM = rightMotorSpeed;
  
  //depending on your connections */
  digitalWrite(INa1, HIGH);
  digitalWrite(INa2, LOW);
  digitalWrite(INb1, HIGH);
  digitalWrite(INb2, LOW);

// to bring back the robot on track based on last error condition //
  if ( (error=0) && (sensor[2]=1)) {
    analogWrite(ENA, leftMotorSpeed+5); //left Motor Speed
    analogWrite(ENB, rightMotorSpeed + 5); 
    digitalWrite(INa1, HIGH);
    digitalWrite(INa2, LOW);
    digitalWrite(INb1, HIGH);
    digitalWrite(INb2, LOW);
  }
  else if ((error < 0) && (sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
  {
    analogWrite(ENA, 63);  //LMotor Speed
    analogWrite(ENB, 57); //R Motor Speed
    digitalWrite(INa1, HIGH);
    digitalWrite(INa2, LOW);
    
    digitalWrite(INb1, HIGH);
    digitalWrite(INb2, LOW);
  

    


  }
  else  if ((error > 0) && (sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
  {
    analogWrite(ENA, 63);  //LMotor Speed
    analogWrite(ENB, 57); //R Motor Speed
    digitalWrite(INa1, HIGH);
    digitalWrite(INa2, LOW);
    digitalWrite(INb1, LOW);
    digitalWrite(INb2, HIGH);


    


  }
 else  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
{
  analogWrite(ENA, 58);  //LMotor Speed
  analogWrite(ENB, 65); //R Motor Speed
  
  
  //depending on your connections */
  digitalWrite(INa1, HIGH);
  digitalWrite(INa2, LOW);
  
  digitalWrite(INb1, LOW);
  digitalWrite(INb2, HIGH);
  
  
  }
else  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
   analogWrite(ENA, 58);  //LMotor Speed
  analogWrite(ENB, 65); //R Motor Speed
  
  
  //depending on your connections */
  digitalWrite(INa1, LOW);
  digitalWrite(INa2, LOW);
  
  digitalWrite(INb1, LOW);
  digitalWrite(INb2, LOW);
}

 


}
