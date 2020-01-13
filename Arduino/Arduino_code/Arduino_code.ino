

//////////////////////////////////////////////////////////////////////
///// TEAM C - Arduino Code - Semi Autonoumous Cargo Robot _v1.5 \\\\\
//////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////


//import Library for QTR IR sensing array
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 5;      // set number of IR LEDs
uint16_t sensorValues[SensorCount]; // initialize sensors

int DIST = 6;                       // set minimum distance of obstacle to stop (in cm)
int vcc = 43;                       //set vcc pin for hc-sr04 (ultrasonic)
int trig = 45;                      //set trigger
int echo = 47;                      //set echo
int gnd = 49;                       //set ground
long dist;
long duration;


////////////////////////Motor Control Pins//////////////////////////
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ena = 5;
int enb = 10;
int error = 0;
int prev_error = 0;
double KP = 0.03;                         //kp, kd
double KD = 0.04;
double kp = 0.02;
double kd = 0.03;
double ki = 0.001;
double integral = 0;
int speedMultiply;                        //variable used to control manual handling speed
char c;
char recieved_pi_Char[2];
int ndx = 0;
int cvt_int;                              // integer to char conversion variable

enum states {MANUAL, AUTO, DEF, STOP, RASP_PI, TURING};   //FSM states

enum states state;


void setup() {

  kp = KP;
  kd = KD;
  Serial2.begin(38400);                        //Initialize Serial for Pi, Bluetooth and Arduino @ 38400
  Serial3.begin(38400);
  Serial.begin(38400);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    40, 42, 44, 46, 48
  }, SensorCount);    //IR array pin definitions
  qtr.setEmitterPin(38);                                                //IR enable PIN
  pinMode(38, OUTPUT);

  //ultrasonic pin intialization
  pinMode(vcc, OUTPUT);
  digitalWrite(vcc, HIGH);

  pinMode(gnd, OUTPUT);
  digitalWrite(gnd, LOW);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);



  //intialize callibration for IR sensor

  for (uint16_t i = 0; i < 10; i++)
  {
    qtr.calibrate();
  }




  int cal = 700;                          //lower bound for intensity
  int cal1 = 1800;                        //upper bound for intensity
  qtr.calibrationOn.minimum[0] = cal;     //manually set intensity instead of relying on on auto-calib
  qtr.calibrationOn.minimum[1] = cal;     //set each LED in IR array with max and min vals
  qtr.calibrationOn.minimum[2] = cal;
  qtr.calibrationOn.minimum[3] = cal;
  qtr.calibrationOn.minimum[4] = cal;

  qtr.calibrationOn.maximum[0] = cal1;
  qtr.calibrationOn.maximum[1] = cal1;
  qtr.calibrationOn.maximum[2] = cal1;
  qtr.calibrationOn.maximum[3] = cal1;
  qtr.calibrationOn.maximum[4] = cal1;


  Serial.println();

  Serial.println();
  Serial.println();
  delay(1000);

  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);

  /////////////////////////////
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  speedMultiply = 55 + 20 * 2;                //initialaze base speed during manual control, slider used to adjust speed

}

void loop() {

  checkCollision();                           //method to detect obstacles


  
  if (Serial2.available()) {                  //manual control is handled here

    c = Serial2.read();

    if (c == 'p') {
      state = AUTO;                           //switch state to autonoumous control
      Serial.println("Auto state initiated");
    }
    if (c == 'q') {
      state = MANUAL;                         //swtich to manual remote control
    }


    if (isdigit(c)) {

      speedMultiply = 55 + 20 * (c - '0');     //write PWM speed to motors depending on slider val
      analogWrite(ena, speedMultiply);
      analogWrite(enb, speedMultiply);

    }
    else {
      switch (c) {
                                              
        case 'a' : forward(speedMultiply); break;   // move forward
        case 'b' : stop1(); break;                  //stop
        case 'r' : rightFine(); break;              //turn right 
        case 'l' : leftFine(); break;               //turn left
        case 'D' : defaultD(speedMultiply); break;  //default speed forward
        case 'u' : kp = kp + 0.01; break;           //increment kp by 0.01
        case 'j' : kp = kp - 0.01; break;
        case 'i' : kd = kd + 0.01; break;           //increment kd by 0.01
        case 'k' : kd = kp - 0.01; break;
        case 'm' : kp = KP; kd = KD; break;         //set default kp/kd
        default : break;

      }

    }


  }

  switch (state) {                  //Finite State Machine 



    case AUTO: irControl(); break;                                //Line following mode                  
    case MANUAL: Serial.println("manual control"); break;         //Manual Remote Control
    case STOP : Serial.println("stopped"); break;                 //stop mode
    case RASP_PI: RaspiSerial(); break;                           // Color detection/ master-slave mode
    case TURING: turning(); break;                                //color detection turning at junction
    default: state = MANUAL ; Serial.println("def state initiated"); break;   //default state


  }





}

/////////////////////////////Autonoumous Line Following ///////////////////////////////////
void irControl() {

  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);                                      //display current reading
    Serial.print('\t');
  }
  Serial.print(position);
  Serial.print('\t');

  delay(15);

  error = (2000 - position);                                            //setpoint 2000

  int speedMotor = kp * error + kd * (error - prev_error);              //error calculation 

  MotorControl(speedMotor);                                             //spin motors depending on error

  prev_error = error;





}

////////////////////////////////For debugging////////////////////
void MotorA(int speed_error) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, speed_error);
  analogWrite(ena, speed_error);

}

void MotorB(int speed_error) {

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, speed_error);
  analogWrite(enb, speed_error);

}
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////// Control Motors Based on Received Error - PD CONTROL///////////////////
int motorAspeed;
int motorBspeed;
int base1 = 90;

void MotorControl(int speed_error) {
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  if (speed_error > -base1 && speed_error < base1) {       //limit speed to prevent unwanted jerks etc


    motorAspeed = base1 - speed_error;
    motorBspeed = base1 + speed_error;

    if (motorAspeed > base1) {
      motorAspeed = base1;
    }
    else if (motorAspeed < 0) {
      motorAspeed = 0;
    }
    if (motorBspeed > base1) {
      motorBspeed = base1;
    }
    else if (motorBspeed < 0) {
      motorBspeed = 0;
    }

    analogWrite(ena, motorAspeed);
    analogWrite(enb, motorBspeed);

    checkBlack();                                           //check for all black (junction)

    Serial.print(speed_error);
    Serial.print('\t');
    Serial.print(motorAspeed);
    Serial.print('\t');
    Serial.println(motorBspeed);

  }



}


//////////////////////////// check for black junction/////////////////////
void checkBlack() {
  int count = 0;
  
  for (int val : sensorValues) {        //check if 4/5 sensors is black

    if (val > 700) {
      count++;
      if (count > 4) {
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        state = TURING;

      }

    }
  }
}



///////////////////////////debugging////////////////////
void checkCorner() {
  int count = 0;


  if (sensorValues[0] > 700 && sensorValues[1] > 700) {

    leftFine();
    delay(400);
    Serial.println("takingright");
  }



}
//////////////////////////////////////////////////////


////////move forward//////////////////////////////////
void forward(int c) {

  //MOTOR A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, c);
  //MOTOR B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, c);
  Serial.println(speedMultiply);

}

/////////////////stop//////////////////////////////////
void stop1() {
  //MOTOR A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);
  //MOTOR B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enb, 0);
  Serial.println("stopped");


}

/////////////////right//////////////////////////////////
void rightFine() {
  analogWrite(ena, 90);
  analogWrite(enb, 30);

}
/////////////////left//////////////////////////////////
void leftFine() {
  analogWrite(enb, 90);
  analogWrite(ena, 30);
}
/////////////////defualt////////////////////////////////
void defaultD(int c) {
  analogWrite(ena, c);
  analogWrite(enb, c);
}

////////////////////////////RASPBERRY PI SERIAL COMMUNICATION//////////////////////////////////////
double Rkp = 12;
double Rkd = 10;
int prev_error_object;

void RaspiSerial() {
  //    Serial.println("Raspi");

  if (Serial3.available()) {

    char str = Serial3.read();
    if (str == 's') {                           // stop if no color is detected
      stop1();

    }
    else if (isdigit(str)) {                    //navigate to "parking garage"
      recieved_pi_Char[0] = str;
      //ndx++;

      //if(ndx==2){

      //  recieved_pi_Char[ndx]='\0';
      sscanf(recieved_pi_Char, "%d", &cvt_int);


      int object_frame_error = 5 - cvt_int ;      


      int speedMotor_PI = Rkp * object_frame_error + Rkd * (error - prev_error);  //positional error of object of interest

      //   MotorControl(speedMotor);
      MotorControl_PI(speedMotor_PI);

      prev_error_object = object_frame_error;
      Serial.print("char");
      Serial.println(recieved_pi_Char);


     }


    }
  }

int base = 80;
//////////////////////////PI_MOTOR CONTROL/////////////////////////////////

int motorAspeed_PI;
int motorBspeed_PI;
void MotorControl_PI(int speed_error) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  if (speed_error > -base && speed_error < base) {


    motorAspeed = base - speed_error;
    motorBspeed = base + speed_error;

    if (motorAspeed > base) {                     //limit speed to prevent unwanted jerks or sudden movements
      motorAspeed = base;
    }
    else if (motorAspeed < 0) {
      motorAspeed = 0;
    }
    if (motorBspeed > base) {
      motorBspeed = base;
    }
    else if (motorBspeed < 0) {
      motorBspeed = 0;
    }

    analogWrite(ena, motorAspeed);
    analogWrite(enb, motorBspeed);


    //checkCorner();
    Serial.print(speed_error);
    Serial.print('\t');
    Serial.print(motorAspeed);
    Serial.print('\t');
    Serial.println(motorBspeed);


  }



}

//////////////////////////TURNING////////////////////////////

int Rcount = 0;
int writeCount = 0;
void turning() {
  writeCount++;
  if (writeCount > 500) {
    Serial3.print('y');
    writeCount = 0;
  }


  if (Serial3.available()) {

    char str = Serial3.read();
    if (str == 'b' || str == 'g' || str == 'y' ) {
      for (int i = 0; i < 8; i++) {
        char c = Serial3.read();
        delay(1000);

        Serial.println(c);
        if (colorCheck(c)) {
          Serial.println(Rcount);
          Rcount++;
        }

        if (Rcount > 4) {
          directionSelect(c);
          Rcount = 0;
          break;
        }

      }
    }


  }
}


char prev_char;
bool colorCheck(char c) {

  if (c == prev_char) {

    return true;
  }
  else {
    prev_char = c;
    return false;
  }


}

void directionSelect(char str) {

  switch (str) {

    case 'b': leftSpecial(100); Serial.println("blue"); break;
    case 'g': rightSpecial(100); Serial.println("green"); break;
    case 'y' : goStraight(100); Serial.println("yellow"); break;
    default : Serial.println("nostring"); break;
  }


  delay(900);
  forward(5);
  stop1();
  state = RASP_PI;


}


void leftSpecial(int c) {
  Serial.println("blue");
  //MOTOR A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enb, c);


  //MOTOR B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);



  analogWrite(ena, c);



}
void rightSpecial(int speed1) {
  Serial.println("right");
  //MOTOR A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //MOTOR B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);


  analogWrite(enb, speed1);
  analogWrite(ena, speed1);

}

void goStraight(int speed1) {
  Serial.println("straight");
  forward(5);

}



long getDistance() {

  digitalWrite(trig, LOW);
  delayMicroseconds(5);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);

  dist = duration * 0.032 / 2;

  return dist;

}
int distVerify = 0;

void checkCollision() {

  dist = getDistance();
  if (dist > 50) {
    return;
  }
  Serial.print("Distance " );
  Serial.println(dist);

  if (dist < DIST) {

    distVerify++;


    if (distVerify > 10) {

      while (dist < DIST) {
        dist = getDistance();
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("under");
      }
      distVerify = 0;

    }

  }



}





