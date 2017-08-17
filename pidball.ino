#include "MeOrion.h"
#include <PID_v1.h>
#include "controls.h"
#include "MeJoystick.h"

// 25 = up ball rolls toward 0
// 162 = down ball rolls to higher value
// 90 is roughly neutral

MeUltrasonicSensor ultraSensor(PORT_4); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

/* Servo */
Servo myservo1; 
MePort port(PORT_3);
int16_t servo1pin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object

double Setpoint, Distance, NewDistance, servo_position;

/* Joystick */
MeJoystick joystick(PORT_7);
int16_t x = 0;    /* a variable for the Joystick's x value */
int16_t y = 0;    /* a variable for the Joystick's y value */
char joystick_wait = 5;


PID myPID(&Distance, &servo_position, &Setpoint,2,5,1,P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                                       //P_ON_E (Proportional on Error) is the default behavior
Me7SegmentDisplay disp(PORT_6);

/* MENU */
ProportionalControl proportional(&myPID, &disp);
IntegralControl integral(&myPID, &disp);
DerivitiveControl derivitive(&myPID, &disp);
PIDControl pid_control(&myPID, &disp, &servo_position);
SetPointControl set_point_control(&myPID, &disp, &Setpoint);
MeasurementControl measurement_control(&myPID, &disp, &Distance);
BallControl *currentControl = &proportional;

void printStatus(PID * myPID){
  double Kp = myPID->GetKp();
  Serial.print("P: ");
  Serial.print(Kp);
  Serial.print(", I: ");
  Serial.print(myPID->GetKi());
  Serial.print(", D: ");
  Serial.println(myPID->GetKd());    
}

void setup()
{
  Serial.begin(9600);
  myservo1.attach(servo1pin);  // attaches the servo on servopin1
  Distance = analogRead(0);
  NewDistance = Distance;
  Setpoint = 15;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(25, 162);
  myPID.SetTunings(1.0, 2.0, 1.0);
  myPID.SetSampleTime(100);

  disp.init();
  disp.set(BRIGHTNESS_2);
  proportional.p_next = &integral;
  proportional.p_prev = &measurement_control;

  integral.p_next = &derivitive;
  integral.p_prev = &proportional;

  derivitive.p_next = &pid_control;
  derivitive.p_prev = &integral;

  pid_control.p_next = &set_point_control;
  pid_control.p_prev = &derivitive;

  set_point_control.p_next = &measurement_control;
  set_point_control.p_prev = &pid_control;

  measurement_control.p_next = &proportional;
  measurement_control.p_prev = &set_point_control;
}

void loop()
{
  NewDistance = ultraSensor.distanceCm();
  Serial.print("New Distance : ");
  Serial.print( NewDistance );
  Serial.println(" cm");
  printStatus(&myPID);
  currentControl->display();
  
  if (NewDistance < 40){
      Distance = NewDistance;
  }
  myPID.Compute();
  myservo1.write(servo_position);

  if (joystick_wait == 0){
    x = joystick.readX();
    y = joystick.readY();

    if ( y > 100 && y < 250){
        currentControl->increase(0.1);
    } else if (y > 250){
        currentControl->increase(1.0);
    } else if (y < -100 && y > -250){
        currentControl->decrease(0.1);
    } else if (y < -250){
        currentControl->decrease(1.0);
    } else if (x > 250){
        currentControl = currentControl->p_next;
    } else if (x < -250){
        currentControl = currentControl->p_prev;
    }

    joystick_wait = 3;
  } else {
      joystick_wait--;
  }
  /* print the results to the serial monitor: */
  delay(100);                           // waits for the servo to get there 


}

