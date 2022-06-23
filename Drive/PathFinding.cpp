#include <Arduino.h>
#include <Robojax_L298N_DC_motor.h>
#include <string>
#include "SPI.h"

#include <iostream>
#include <iterator>
#include <list>
using namespace std;


// motor 1 right wheel
#define CHA 0
#define ENA 22 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 15
#define IN2 21
// motor 2 settings left wheel
#define IN3 2
#define IN4 16
#define ENB 4 // this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW = 1;  // do not change
#define motor1 1   // do not change
#define motor2 2   // do not change

// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);

#define PIN_SS 5
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18

#define PIN_MOUSECAM_RESET 17
#define PIN_MOUSECAM_CS 5

#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30

#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1d
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60

#define ADNS3080_PRODUCT_ID_VAL 0x17

//Values for the Rover
int headroom = 100; // prevent hitting the wall

float length_r = 200, width_r = 200, increment; //size of the rover
float length_c = 4000, width_c = 3000; //size of the court

// Values for optical flow sensor
int total_x = 0;
int total_y = 0;

int total_x1 = 0;
int total_y1 = 0;

int x = 0;
int y = 0;

int a = 0;
int b = 0;

int distance_x = 0;
int distance_y = 0;

volatile byte movementflag = 0;
volatile int xydat[2];

// Values for modes
char mode = 'C';
char motion = 'S';
char mode = 'M';
char motion = 'D';
boolean reachDestination = false;
int destination_x = 7;
int destination_y = 7;
int destination_x_prev = 0;
int destination_y_prev = 0;

// Values for PID
float kp = 0.7;
float ki = 0;
float kd = 0;
float controlSignal = 0;
float prevT = 0;     // for calculating delta t
float prevE = 0;     // for calculating the derivative (edot)
float eIntegral = 0; // integral error
float currT = 0;     // time in the moment of calculation
float deltaT = 0;    // time difference
float error = 0;     // error
float edot = 0;      // derivative (de/dt)

// Values for controller
int desire_x, desire_y;
float dy_mm = 0, dx_mm = 0;
int current_x = 0, current_y = 0;
int constant = 0;
float current_angle = 0;
int r = 144;
float control_done = 0;

// Values for mapping
int i, period;
float error_distance;
float error_angle;
bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = true;
bool turn_90_1 = false, turn_90_2 = false, turn_90_3 = false, turn_90_4 = false;
bool turn_done_5, move_done_5, turn_done_6, move_done_6;
bool check_divert_done = true;
bool update_destination_xy_1, update_destination_xy_2 = true;
bool back2trace;

// Values for check_detect
int binary_output;
float d_o;
float angle_o;
float del_x;
float del_y;
float safe_d = 200;
float change_x,change_y;
bool detected;

// Values for the coordinate_list
bool check_done;//also in mode 'C'
int follow = 1;
typedef list<float> Coordinate_list;
Coordinate_list coordinate_list;
Coordinate_list::iterator it;
float x1,x2,y1,y2;
float del_x;
float OH, base;
float final_destination_x, final_destination_y;

// Values for timer
long lastTrigger = 0;
boolean startTimer = false;
const int motionSensor = 18; // in adns3080

////FUNCTIONS////
void IRAM_ATTR detectsCLK()
{
  Serial.println("SCLK rising");
  digitalWrite(20, HIGH); // pin20 for debug only
  bool startTimer = true;
  lastTrigger = micros();
}

int convTwosComp(int b)
{
  // Convert from 2's complement
  // only negative number will go through this
  if (b & 0x80)
  {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}

int tdistance = 0;

void mousecam_reset()
// reset the sensor to restore it to normal motion
{
  digitalWrite(PIN_MOUSECAM_RESET, HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET, LOW);
  delay(35); // 35ms from reset to functional
}

int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET, OUTPUT);
  pinMode(PIN_MOUSECAM_CS, OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS, HIGH);

  mousecam_reset();
  return 1;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80); // MSB is '1' to indicate data direction
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff); // send a dummy byte to the EEPROM for the purpose of shifting the data out
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(1);
  return ret;
}

struct MD
{
  byte motion;
  char dx, dy;
  byte squal;
  word shutter;
  byte max_pix;
};

void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion = SPI.transfer(0xff);
  p->dx = SPI.transfer(0xff);
  p->dy = SPI.transfer(0xff);
  p->squal = SPI.transfer(0xff);
  p->shutter = SPI.transfer(0xff) << 8;
  p->shutter |= SPI.transfer(0xff);
  p->max_pix = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE, 0x83);
  // write 0x83 to this register will cause the next available complete 1 2/3 frames of pixel values to be stored to SROM RAM
  // write to this register is required before using the Frame Capture brust mode to read the pixel values

  digitalWrite(PIN_MOUSECAM_CS, LOW);

  SPI.transfer(ADNS3080_PIXEL_BURST); // used for high-speed access to all the pixel values from one and 2/3 complete frame
  delayMicroseconds(50);

  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for (count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y;)
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if (started == 0)
    {
      if (pix & 0x40) // the first pixel of the first frame has bit 6 set to 1 as a start-of-frame marker
        started = 1;
      else
      {
        timeout++;
        if (timeout == 100)
        {
          ret = -1;
          break;
        }
      }
    }
    if (started == 1)
    {
      pdata[count++] = (pix & 0x3f) << 2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(14);

  return ret;
}

float calculatePID(float error)
{
  // Determining the elapsed time
  currT = micros();                     // current time
  deltaT = (currT - prevT) / 1000000.0; // time difference in seconds
  prevT = currT;                        // save the current time for the next iteration to get the time difference

  //---
  edot = (error - prevE) / deltaT; // edot = de/dt - derivative term

  eIntegral = eIntegral + (error * deltaT); // integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (kp * error) + (kd * edot) + (ki * eIntegral); // final sum, proportional term also calculated here

  prevE = error; // save the error for the next iteration to get the difference (for edot)
  currT = micros();
  return controlSignal;
}

bool distance_control(int desired_x, int desired_y)
{
  float error_distance = sqrt(pow((current_x - desired_x), 2) + pow((current_y - desired_y), 2));
  float control_distance = calculatePID(error_distance);
  // Determine speed and direction based on the value of the control signal
  // direction
  if (error_distance > 3 && control_distance < 0) // move backward
  {
    constant = -1; // move backwards, when calculate current position should minus the distance it traveld
    if (error_distance > 100)
    {
      robot.rotate(motor1, (abs(control_distance) * 0.01 + 0.5) * 100, CCW);
      robot.rotate(motor2, (abs(control_distance) * 0.01 + 0.5) * 100, CCW);
    }
    else if (error_distance > 50)
    { // move backward with 40 speed when error distance in range 100-50
      robot.rotate(motor1, 40, CCW);
      robot.rotate(motor2, 40, CCW);
    }
    else // move backward with 20 speed when error distance less than 50
    {
      robot.rotate(motor1, 20, CCW);
      robot.rotate(motor2, 20, CCW);
    }
  }
  else if (error_distance > 3 && control_distance > 0) // move forward
  {
    constant = 1; // move backwards, when calculate current position should minus the distance it traveld
    if (error_distance > 100)
    {
      robot.rotate(motor1, (abs(control_distance) * 0.01 + 0.5) * 100, CW);
      robot.rotate(motor2, (abs(control_distance) * 0.01 + 0.5) * 100, CW);
    }
    else if (error_distance > 50)
    { // move backward with 40 speed when error distance in range 100-50
      robot.rotate(motor1, 40, CW);
      robot.rotate(motor2, 40, CW);
    }
    else // move backward with 20 speed when error distance less than 50
    {
      robot.rotate(motor1, 20, CW);
      robot.rotate(motor2, 20, CW);
    }
  }
  else // == 0, stop/break
  {
    robot.brake(1);
    robot.brake(2);
    return true;
    // In this block we also shut down the motor and set the PWM to zero
  }

  float measured_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
  current_y = current_y + (constant * measured_distance) * cos(current_angle);
  current_x = current_x + (constant * measured_distance) * sin(current_angle);
  return false;
}

bool angel_control(float desired_angle, float dy, float dx, float *current_an)
{
  float error_angle = desired_angle - current_angle;
  float control_angle = calculatePID(error_angle);
  // Determine speed and direction based on the value of the control signal
  // direction
  if (abs(error_angle) >= 2 && error_angle >= 2) // turn right
  {
    if (abs(error_angle) > 10)
    {
      robot.rotate(motor1, (abs(control_angle) * 0.01 + 0.5) * 100, CCW);
      robot.rotate(motor2, (abs(control_angle) * 0.01 + 0.5) * 100, CW);
    }
    else if (abs(error_angle) > 5)
    { // move backward with 40 speed when error distance in range 100-50
      robot.rotate(motor1, 40, CCW);
      robot.rotate(motor2, 40, CW);
    }
    else // move backward with 20 speed when error distance less than 50
    {
      robot.rotate(motor1, 20, CCW);
      robot.rotate(motor2, 20, CW);
    }
  }
  else if (abs(error_angle) >= 2 && error_angle <= -2) // turn left
  {
    if (abs(error_angle) > 10)
    {
      robot.rotate(motor1, (abs(control_angle) * 0.01 + 0.5) * 100, CW);
      robot.rotate(motor2, (abs(control_angle) * 0.01 + 0.5) * 100, CCW);
    }
    else if (abs(error_angle) > 5)
    { // move backward with 40 speed when error distance in range 100-50
      robot.rotate(motor1, 40, CW);
      robot.rotate(motor2, 40, CCW);
    }
    else // move backward with 20 speed when error distance less than 50
    {
      robot.rotate(motor1, 20, CW);
      robot.rotate(motor2, 20, CCW);
    }
  }
  else // == 0, stop/break
  {
    robot.brake(1);
    robot.brake(2);
    return true;
  }

  float measured_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
  float moved_angle = asin(measured_distance / (2 * r)) * 2 * (180 / 3.14159265359);
  float angle = *current_an;
  angle = (dx < 0) ? (angle + moved_angle) : (angle - moved_angle);
  angle = (angle > 180) ? (angle - 180) : angle;
  angle = (angle < -180) ? (angle + 180) : angle;
  *current_an = angle;
  return false;
}

float convertTodegree(float angle_radians)
{
  return angle_radians * (180 / 3.14159265359);
}

void setup()
{
  //Serial.begin(9600);

  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  robot.begin();

  Serial1.begin(115200);

  pinMode(motionSensor, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsCLK, RISING);

  Serial2.begin(9600);
  if (mousecam_init() == -1)
  {
    Serial.println("Mouse cam failed to init");
    while (1)
      ;
  }
}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k >> 4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

void loop()
{
#if 0
/*
    if(movementflag){

    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag=0;
    delay(3);
    }

  */
  // if enabled this section grabs frames and outputs them as ascii art

  if(mousecam_frame_capture(frame)==0)
  {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
    {
      for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  delay(250);

#else

  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.
  // move straight for 3 sec

  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM); // find the avrage pixel value
  MD md;
  mousecam_read_motion(&md);
  for (int i = 0; i < md.squal / 4; i++) // number of features = SQUAL register value *4
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val * 100) / 351); // calculate average pixel
  Serial.print(' ');
  Serial.print(md.shutter);
  Serial.print(" (");
  Serial.print((int)md.dx);
  Serial.print(',');
  Serial.print((int)md.dy);
  Serial.println(')');

  // Serial.println(md.max_pix);// maximum = 63
  delay(100);

  distance_x = md.dx; // convTwosComp(md.dx);
  distance_y = md.dy; // convTwosComp(md.dy);

  total_x1 = total_x1 + distance_x;
  total_y1 = total_y1 + distance_y;

  total_x = (total_x1 / 157) * 10;
  total_y = (total_y1 / 157) * 10;

  dx_mm = (distance_x / 157) * 10; // convert distance to mm
  dy_mm = (distance_y / 157) * 10;

  Serial.print('\n');

  Serial.println("Distance_x = " + String(total_x));

  Serial.println("Distance_y = " + String(total_y));
  Serial.print('\n');

  delay(250);

  /// Modes for command from Vision////
  if (mode == 'V')
  {
    if (motion == 'F')
    {
      robot.rotate(motor1, 60, CW);
      robot.rotate(motor2, 60, CW);
    }
    else if (motion == 'B')
    {
      robot.rotate(motor1, 60, CCW);
      robot.rotate(motor2, 60, CCW);
    }
    else if (motion == 'L')
    {
      robot.rotate(motor1, 60, CW);
      robot.rotate(motor2, 60, CCW);
    }
    else if (motion == 'R')
    {
      robot.rotate(motor1, 60, CCW);
      robot.rotate(motor2, 60, CW);
    }
    else if (motion == 's')
    {
      robot.brake(1);
      robot.brake(2);
    }
  }

  float measured_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
  float direction = (dy_mm > 0) ? 1 : -1; // if dy_mm>0 Rover goes forward
  current_y = current_y + (direction * measured_distance) * cos(current_angle);
  current_x = current_x + (direction * measured_distance) * sin(current_angle);

  //float measured_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
  float moved_angle = asin(measured_distance / (2 * r)) * 2 * (180 / 3.14159265359);
  current_angle = (dx_mm < 0) ? (current_angle + moved_angle) : (current_angle - moved_angle);
  current_angle = (current_angle > 180) ? (current_angle - 180) : current_angle;
  current_angle = (current_angle < -180) ? (current_angle + 180) : current_angle;

  if (mode == 'C')
  {
    reachDestination = (destination_x != destination_x_prev || destination_y != destination_y_prev) ? false : reachDestination;
    float change_x = destination_x - current_x;
    float change_y = destination_y - current_y;
    bool turn_done;
    bool move_done;
    bool check_done;
    if (destination_x > 1000 || destination_y > 1000)
    { // not receiving correct destination position yet
      robot.brake(1);
      robot.brake(2);
    }
    else
    {
      // rotate first
      if (change_x > 0 && change_y > 0)
      {
        turn_done = angel_control(convertTodegree(atan(change_x / change_y)), dy_mm, dx_mm, &current_angle); // 1st quadrant
      }
      else if (change_x < 0 && change_y > 0)
      {
        turn_done = angel_control(convertTodegree(atan(change_x / change_y)), dy_mm, dx_mm, &current_angle); // 2nd quadrant
      }
      else if (change_x < 0 && change_y < 0)
      {
        turn_done = angel_control(convertTodegree(atan(-change_y / change_x)) - 90, dy_mm, dx_mm, &current_angle); // 3rd quadrant
      }
      else if (change_x > 0 && change_y < 0)
      {
        turn_done = angel_control(convertTodegree(atan(-change_y / change_x)) + 90, dy_mm, dx_mm, &current_angle); // 4th quadrant
      }

      // move forward/backward
      if (turn_done && !reachDestination)
      {
        move_done = distance_control(destination_x, destination_y);
        check_done = reachDestination;
      }
      // arrive destination reset destination position
      if (check_done)
      {
        destination_x = 8888;
        destination_y = 8888;
      }
    }
  }

  bool start = true;
  if (start){   
    int headroom = 150; // prevent hitting the wall
    int increment = 0; //in declearation

    float length_r = 200, width_r = 200; //size of the rover[mm]
    float length_c = 4000, width_c = 3000; //size of the court

    period =  (width_c-2*headroom) / (3*width_r);
    //bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = true;
    //bool turn_90_1 = false, turn_90_2 = false, turn_90_3 = false, turn_90_4 = false;

    for(i; i <= period; i++){


      if(!go_straight_1 && go_straight_4){

        destination_x = 20+increment;
        destination_y = length_c-headroom-length_r;
        cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
        mode = 'C';
        if(reachDestination){
          go_straight_1 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_1 && !go_straight_2){
        
        increment = increment + 200;
        destination_x = 200+increment;
        destination_y = length_c-headroom-length_r;
        cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
        mode = 'C';
        if(reachDestination){
          go_straight_2 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_1 && !go_straight_2){
        
        increment = increment + 200;
        destination_x = 200+increment;
        destination_y = headroom+length_r;
        mode = 'C';
        cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
        if(reachDestination){
          go_straight_3 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_3 && !go_straight_4){

        if(i = period){
          destination_x = 200+increment;
          destination_y = headroom+length_r;
          mode = 'C';
          cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
          if(reachDestination){
            go_straight_4 = true;
            return;
          }
        }

        increment = increment + 200;
        destination_x = 200+increment;
        destination_y = headroom+length_r;
        mode = 'C';
        cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
        if(reachDestination){
          go_straight_4 = true;
          cout << "Detection completed." << endl;
          reachDestination = false;
          go_straight_1 = false;
        }        
      }
      
    }
  }

 
  /*if (mode == 'M'){  //for mapping
    int headroom = 100; // prevent hitting the wall
    int increment = 0; //in declearation

    float length_r = 200, width_r = 200; //size of the rover[mm]
    float length_c = 4000, width_c = 3000; //size of the court

    period =  (width_c-200) / (2*width_r);
    //bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = true;
    //bool turn_90_1 = false, turn_90_2 = false, turn_90_3 = false, turn_90_4 = false;

    for(i; i <= period; i++){


      if(!go_straight_1 && go_straight_4){

        int binary_output;//to be confirmed, from Vision
        
        //check whether D state is high or not(check_divert_done = 1)
        detected = check_detect(binary_output);

        //if (detected || !check_divert_done)
        divert(error_distance, error_angle);
        
        
        if(check_divert_done){
          go_straight_1 = distance_control(200+increment, 4000-headroom-length_r);
        }
      }

      else if(go_straight_1 && !turn_90_1){

        turn_90_1 = angel_control(90, dy_mm, dx_mm, &current_angle);
      }

      else if(turn_90_1 && !go_straight_2){
        increment = increment + 200;
        go_straight_2 = distance_control(200+increment, 4000-headroom-length_r);
      }
    
      else if(go_straight_2 && !turn_90_2){
        turn_90_2 = angel_control(180, dy_mm, dx_mm, &current_angle);
      }

      else if(turn_90_2 && !go_straight_3){
  
        go_straight_3 = distance_control(200+increment, destination_y_prev);//y no change
      }

      else if(go_straight_3 && !turn_90_3){
        if(i = period){
          turn_90_3 = angel_control(-90, dy_mm, dx_mm, &current_angle);
          if(turn_90_3 && !go_straight_4){
            go_straight_4 = distance_control(200, 300);// back to base
            if(go_straight_4){
              break;
            }
          }
        }
        turn_90_3 = angel_control(90, dy_mm, dx_mm, &current_angle);
      }

      else if(turn_90_3 && !go_straight_4){
        increment = increment + 200;
        go_straight_4 = distance_control(200+increment, headroom+length_r);
      }

      else{
        turn_90_4 = angel_control(0, dy_mm, dx_mm, &current_angle);
        go_straight_1 = go_straight_2 = go_straight_3 = go_straight_4 = false;
        turn_90_1 = turn_90_2 = turn_90_3 = false;
      }
    }

  }*/

  //如果detect到障碍物，vision output持续HIGH，state D=1，执行divert

  currT = micros();
  float sendback = 100; 
  if (startTimer && (currT - lastTrigger > (sendback * 1000))) // Sends info every 100 ms
  {

    Serial.println("Last trigger time: " + String(lastTrigger));
    Serial.println("Current time: " + String(currT));

    digitalWrite(20, LOW);
    startTimer = false;

    lastTrigger = currT;
    //sendcoords(current_x, current_y, current_angle); // Sends info to ESP

    detachInterrupt(motionSensor);
    Serial.println("Interrupt Detached");
  
    int headroom = 100; // prevent hitting the wall

    float length_r, width_r = 200, gap = 200; //size of the rover
    float length_c = 4000, width_c = 3000; //size of the court
    

    int circle =  (width_c-200) / (2*width_r);
    
  }

#endif
}


//wheather the obstacle block the way?  if yes,(del_x < width_r)
//otherwise, only rotate to find the angle
bool found; //signal given from the Vision
float send_x, send_y;
bool send_coordinate;

void rotate_v(bool found, bool detect_d_o, float &send_x, float &send_y){
  //this function is called when the obstacle is detected
  //but no reroute is required
  if(found){
    float d_o = detect_d_o;
    send_coordinate = 1;
    send_x = current_x + sin(current_angle);
    send_y = current_y + cos(current_angle);
    send_coordinate = 0;
    mode = 'C';
    //the destination is given by either the default path or the deflection
    //rotate back to the original driving angle
  }
  else{
    if(-88 < angle_o && angle_o < -2){
      mode = 'V';
      motion = 'L';
    }
    else if(88 > angle_o && angle_o > 2){
      mode= 'V';
      motion = 'R';
    }
  }
}

float pi, desire_angle;
void rotate360(){
  float desired_angle =pi/2;
  bool turn_done;
  while(desire_angle<2*pi){
   turn_done = angel_control(convertTodegree(desire_angle), dy_mm, dx_mm, &current_angle);
   desired_angle=desired_angle+pi;
  }
}

void rotate_r(float dc_voltage, float &send_x, float &send_y){
  float orientation;
  return;
}

    

void divert(float d_o, float angle_o){


    if (detected || !check_divert_done){
      Serial.println("An obstacle detected! ");

      //check_divert_done = false;
      //bool update_destination_xy;
      //float d_o;// distance from the rover to the obstacle detected, given from the Vision
      //float angle_o;//angle between the current 'go straight' path and the line connected the rover and the obstacle

      //float del_x = sin(angle_o)*d_o;
      //float del_y = cos(angle_o)*d_o;

      if((del_x < width_r) && (current_y + 2*del_y < length_c-length_r-headroom)){ //limit due to court length
      
        check_divert_done = false;

        float safe_d = 200;
        //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
        float div_angle = current_angle + angle_o + convertTodegree(asin(safe_d / d_o));

        back2trace = (destination_x != destination_x_prev || destination_y != destination_y_prev) ? false : back2trace;
        //float change_x = destination_x - current_x;
        //float change_y = destination_y - current_y;
        //bool turn_done_5, move_done_5, turn_done_6, move_done_6, check_divert_done;

        // rotate first
        if (!turn_done_5 && !back2trace){
          if (change_x > 0 && change_y > 0)
          {
            turn_done_5 = angel_control(convertTodegree(atan(change_x / change_y)), dy_mm, dx_mm, &current_angle); // 1st quadrant
          }
          else if (change_x < 0 && change_y > 0)
          {
            turn_done_5 = angel_control(convertTodegree(atan(change_x / change_y)), dy_mm, dx_mm, &current_angle); // 2nd quadrant
          }
          else if (change_x < 0 && change_y < 0)
          {
            turn_done_5 = angel_control(convertTodegree(atan(-change_y / change_x)) - 90, dy_mm, dx_mm, &current_angle); // 3rd quadrant
          }
          else if (change_x > 0 && change_y < 0)
          {
            turn_done_5 = angel_control(convertTodegree(atan(-change_y / change_x)) + 90, dy_mm, dx_mm, &current_angle); // 4th quadrant
          }
        }

        // move forward/backward
        if (turn_done_5 && !move_done_5)
        {
          if(turn_90_1 && (current_y + 2*del_y > length_c-length_r-headroom)){
            go_straight_1 = true;
            return;
          }
          if(turn_90_2 && current_y < 2*del_y +length_r+headroom){ //go_straight_1 && turn_90_1 && go_straight_1 &&
            go_straight_3 = true;
            return;
          }

          bool new_detect = check_detect(binary_output);

          if (new_detect){
            Serial.println("New obstacle detected! ");
            return;
          }

          move_done_5 = distance_control(destination_x, destination_y);
        }
          
        if (move_done_5 && !turn_done_6){

          if(!update_destination_xy_2){
            //move from (x+del_x+del_y, y+del_y) to (x, y+2*del_y)
            destination_x = current_x - del_x - safe_d/(cos(div_angle));
            destination_y = current_y + del_y;
            update_destination_xy_2 = true;
          }

          change_x = destination_x - current_x;
          change_y = destination_y - current_y;

          if (change_x > 0 && change_y > 0)
          {
            turn_done_6 = angel_control(convertTodegree(atan(change_x / change_y)), dy_mm, dx_mm, &current_angle); // 1st quadrant
          }
          else if (change_x < 0 && change_y > 0)
          {
            turn_done_6 = angel_control(convertTodegree(atan(-change_x / change_y)), dy_mm, dx_mm, &current_angle); // 2nd quadrant
          }
          else if (change_x < 0 && change_y < 0)
          {
            turn_done_6 = angel_control(convertTodegree(atan(-change_y / change_x)) - 90, dy_mm, dx_mm, &current_angle); // 3rd quadrant
          }
          else if (change_x > 0 && change_y < 0)
          {
            turn_done_6 = angel_control(convertTodegree(atan(-change_y / change_x)) + 90, dy_mm, dx_mm, &current_angle); // 4th quadrant
          }

        }   
            
        if (turn_done_6 && !back2trace){
            
            bool new_detect = check_detect(binary_output);

            if (new_detect){
              Serial.println("New obstacle detected! ");
              return;
            }

            move_done_6 = distance_control(destination_x, destination_y);
            check_divert_done = back2trace;
        }  
      }
    }

    else{
      return; //the state of D is low, do nothing
    }
}


bool check_detect(int binary_output){

  switch(binary_output){
    case 0:
    //check_divert_done = true;
    return false;

    case 1:
    update_destination_xy_1 = false;

    //these two would be input
    float d_o;//  =; distance from the rover to the obstacle detected, given from the Vision
    float angle_o;// =; angle between the current 'go straight' path and the line connected the rover and the obstacle

    float del_x = sin(angle_o)*d_o;
    float del_y = cos(angle_o)*d_o;

    if(del_x < width_r) { // block the way
      back2trace = false;

      float safe_d = 200;
      //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
      float safe_angle = convertTodegree(asin(safe_d / d_o));
      float div_angle = current_angle + angle_o + safe_angle;

      //move from (x,y) to (x+ safe_d/(cos(div_angle)), y+del_y)
      if (!update_destination_xy_1){
        destination_x = current_x + safe_d/(cos(div_angle)); //should be a constant
        destination_y = current_y + del_y;
        update_destination_xy_1 = true;//只有在新的信号（不同的divert信号）出现的时候，改变坐标
        update_destination_xy_2 = false;
      }
     
      back2trace = (destination_x != destination_x_prev || destination_y != destination_y_prev) ? false : back2trace;
      change_x = destination_x - current_x;
      change_y = destination_y - current_y;

      turn_done_5 = false;
      move_done_5 = false;
      turn_done_6 = false;
      move_done_6 = false;
    }
    return true;
  }
}

//make sure the gobal vars update frequently

//2 elements will be added to the list once
//always add coordinates in the middle of the list
typedef list<float> Coordinate_list;

bool generate_coordinates(){
  //only when check_detect is true, generate coor
  if(check_detect){

    float d_o;//  =; distance from the rover to the obstacle detected, given from the Vision
    float angle_o;// =; angle between the current 'go straight' path and the line connected the rover and the obstacle

    del_x = sin(angle_o + current_angle)*d_o;
    OH = cos(angle_o)*d_o;

    float safe_d = 200;
    //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
    float safe_angle = convertTodegree(asin(safe_d / d_o));
    
    float divert_angle, desire_angle;
    if(current_angle < 0 && !go_straight_3 && (i != period)){// divert from left hand side
      divert_angle = (-1)*angle_o + safe_angle; // must be positive according to the setting scheme
      desire_angle = current_angle - divert_angle;
    }

    else{
      divert_angle = angle_o + safe_angle;
      desire_angle = current_angle + divert_angle;
    }
 
    //float desire_angle = current_angle + divert_angle;
    
    float magnitude = OH / cos(divert_angle);
    float base = 2*OH;

    Coordinate_list coordinate_list;
    Coordinate_list::iterator it;
    
    //add one intermediate point
    float x1 = current_x + magnitude * sin(desire_angle);
    float y1 = current_y + magnitude * cos(desire_angle);
    //coordinate_list.push_front(x1);
    //coordinate_list.push_back(y1);
    
    //add second point to finish divesion
    float x2 = current_x + base * sin(current_angle);
    float y2 = current_y + base * cos(current_angle);
    //coordinate_list.push_back(x2);
    //coordinate_list.push_back(y2);

    cout<<"coordinate_list.begin()----coordinate_list.end():" << endl;
    for (it; it != coordinate_list.end(); ++it){
      cout << *it << " ";
    }
    cout << endl;

    return true;

    //2 coordinates generated successfully

    //then we need to go
    //int follow = 1;

    //it = coordinate_list.begin();
    //destination_x = *it;
    //advance(it,follow);
    //destination_y = *it;

    //mode = 'C';
    //call function: move towards A1
    
    for(int count_pair; coordinate_list.size()/2>0; ){
    it = coordinate_list.begin();
    destination_x = *it;
    advance(it,follow);
    destination_y = *it;

    it = coordinate_list.end();
    final_destination_y = *it;
    final_destination_x = *--it;

    }

    mode = 'C';
    //call function
    if(check_done){
      it = coordinate_list.begin();
      coordinate_list.erase(it);
      advance(it,follow);
      coordinate_list.erase(it);
    }

    
    //int order_position;// which pair of coordinates 
  }
  
  return false;
}

void edit_list(){

  if(check_done){
    it = coordinate_list.begin();
    coordinate_list.erase(it);
    advance(it,follow);
    coordinate_list.erase(it);

    return;
  
  }

  if(generate_coordinates){
    coordinate_list.push_front(y2);
    coordinate_list.push_front(x2);
    coordinate_list.push_front(y1);
    coordinate_list.push_front(x1);
  }
  
  go_to_point();
  //already set current destination;
  float d2destination = sqrt(pow(destination_x-current_x,2) + pow(destination_y-current_y,2));

  if(2*OH > d2destination){
    it = coordinate_list.begin();
    coordinate_list.erase(it);
    advance(it,follow);
    coordinate_list.erase(it);

    coordinate_list.push_front(y2);
    coordinate_list.push_front(x2);// 可以和下面的合并
    coordinate_list.push_front(y1);
    coordinate_list.push_front(x1);

  }

  //only when one coordinate left, and we want to add two more 
  if(destination_x < final_destination_x){
    coordinate_list.push_front(y1);
    coordinate_list.push_front(x1);
    //add only one coordinate
    
    it = coordinate_list.end();
    coordinate_list.erase(it);
    coordinate_list.erase(it);
    //last coordinate deleted!
    //then the rover get back to the normal path gradually

  }


  return;
}

void go_to_point(){//执行设置目的地以及要求行动

  for(int count_pair; coordinate_list.size()/2>0; ){
    it = coordinate_list.begin();
    destination_x = *it;
    advance(it,follow);
    destination_y = *it;
  }
  mode = 'C';
}
