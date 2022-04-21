// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016
// Jerry Ng           - jerryng  _ mit _ edu,    Feb  2019

#include "Arduino.h"
#include "helper.h"

EncoderMeasurement  encoder(26);      // FIX THIS: encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
SerialComm          serialComm;       // serial communication class
PathPlanner         pathPlanner;      // path planner
unsigned long       prevTime = 0;

float VL = 0;
float VR = 0;
float V_End_Y = 0;
float V_End_Scoop = 0;

boolean usePathPlanner = false;

void setup() {
    Serial.begin(115200);       // initialize Serial Communication
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    unsigned long currentTime = micros();
    
    if (currentTime - prevTime >= PERIOD_MICROS) {
      
        // 1. Obtain and convert encoder measurement
        encoder.update(); 

        // 2. Compute robot odometry
        robotPose.update(encoder.dPhiL, encoder.dPhiR); 

        // 3. Send robot odometry through serial port
        serialComm.send(robotPose); 
  
        // 4. Compute desired wheel velocity without or with motion planner
        if (!usePathPlanner) {
        // 4.1 wheel speed depends on keys pressed
          if (key == CODED){
            switch(keyCode){
              case UP:
                drive_forwards();
                break;
              case DOWN:
                drive_backwards();
                break;
              case LEFT:
                drive_ccw();
                break;
              case RIGHT:
                drive_cw();
                break;
             }
          } else stall();
          
            pathPlanner.desiredWV_R = VR;   
            pathPlanner.desiredWV_L = VL;
        }
        else{
            // 4.2 compute wheel speed from using a navigation policy
            pathPlanner.navigateTrajU(robotPose); 
        }

        // 5. Command desired velocity with PI controller
        wheelVelCtrl.doPIControl("Left",  pathPlanner.desiredWV_L, encoder.v_L); 
        wheelVelCtrl.doPIControl("Right", pathPlanner.desiredWV_R, encoder.v_R);

        prevTime = currentTime; // update time
    } 
}

void drive_forwards() {
  VL = 100;
  VR = 100;
}

void drive_backwards() {
  VL = -100;
  VR = -100;
}

void drive_cw() {
  VL = 50;
  VR = -50;
}

void drive_ccw() {
  VL = -50;
  VR = 50;
}

void stall(){
  //Setting both wheel velocities to zero, at stopped position
  VL = 0;
  VR = 0;
}

void lift_up() {
  //if not at max height
    //Increase target height by some incremental value
}

void lift_down() {
  //if not at min height
    //Decrease target height by some incremental value
}

void spin_cw() {
  //If not at max theta
    //Increase theta by some value
}

void spin_ccw() {
  //If not at min theta
    Decrease by some value
}
