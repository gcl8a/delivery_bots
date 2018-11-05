/* 
 * File:   robot.h
 * Author: greg
 *
 * Created on 3. november 2012, 22:53
 */

#ifndef __ROBOT_H
#define __ROBOT_H

#include "comm.h"
#include "mc33926.h"
#include "motion_controller.h"
#include "pose.h"

#include <matrix.h>

#define VAR_VEL_ENC 1e-2
#define VAR_POS_GPS 1e-4

#define DESTINATION_TOLERANCE 0.1  //m

 /*
 * Calculation for RADIANS_PER_TICK:
 * 12 magnets = 48 ticks / rotation
 * Gear ratio of 9.7:1 -> 465.6 ticks / wheel rotation 
 * = 465.6 ticks / rot / (0.285 m / rot)
 */

#define TICKS_PER_METER 1520
#define RADIUS_ROBOT 0.114

enum UGV_STATE {DRIVE_IDLE, DRIVE_DEAD_RECK, DRIVE_GPS};

class UGV
{  
protected:
  UGV_STATE robotState = DRIVE_IDLE;
  
  ivector wheel_speeds;
  ivector effort;
  
  MC33926 driver;
  MotionController motionController;

  Pose currPose;
  Pose destPose;
  
  dmatrix P;

  double K_dist = 0.4;
  double K_ang  = 1.0;

  volatile uint8_t readyToReport = 0; //does this need to be volatile?

public:
  UGV(void) : wheel_speeds(2), effort(2), P(3,3)
  {}
  
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("UGV::Init");

    driver.Init(COMM_PWM);
    motionController.Init();

    Idle();

    DEBUG_SERIAL.println("/UGV::Init");
  }

  void Idle(void)
  {
    driver.FullStop();
    robotState = DRIVE_IDLE;
  }

  virtual void MainLoop(void)
  {
    if(CheckDestination())
    {
      ivector zero(2);
      CommandMotors(zero);
      robotState = DRIVE_IDLE;
    }
    
    if(readyToPID) 
    {
      ProcessPID();
      readyToPID = 0;
      readyToReport++;
    }
  }

  void SetDestination(double x, double y)
  {
    destPose.x = x;
    destPose.y = y;

    robotState = DRIVE_DEAD_RECK;
  }

  boolean CheckDestination(void)
  {
    if(robotState != DRIVE_IDLE)
    {
      if(CalcDistanceBetweenPoses(currPose, destPose) < DESTINATION_TOLERANCE) return true;
    }

    return false;
  }

  virtual void ProcessPID(void)
  {
      wheel_speeds = motionController.CalcEstimate(); //wheel velocity is ticks/period

      DEBUG_SERIAL.print(wheel_speeds[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(wheel_speeds[1]);
      DEBUG_SERIAL.print('\t');

      //get the latest prediction (stored in currPose):
      UpdatePrediction(wheel_speeds[0], wheel_speeds[1]);

      if(robotState == DRIVE_DEAD_RECK)
      {
        dvector error(2);
        error[0] = CalcDistanceBetweenPoses(destPose, currPose);

        float angleError = CalcAngleToPose(destPose, currPose) - currPose.theta;
        while(angleError >  M_PI) {angleError -= 2 * M_PI;}
        while(angleError < -M_PI) {angleError += 2 * M_PI;}
        error[1] = angleError;
        
        DEBUG_SERIAL.print(error[0]);
        DEBUG_SERIAL.print('\t');
        DEBUG_SERIAL.print(error[1]);
        DEBUG_SERIAL.print('\t');

        SetTwistSpeed(error[0] * K_dist, error[1] * K_ang);
  
        effort = motionController.CalcEffort();
        CommandMotors(effort);
      }
  }
  
  void SetTwistSpeed(float vel, float ang_vel)
  {
    //do calculations in m/s
    float speedLeft  = vel - ang_vel * RADIUS_ROBOT;
    float speedRight = vel + ang_vel * RADIUS_ROBOT;

    SetWheelSpeeds(speedLeft, speedRight);
  }

  void SetWheelSpeeds(float speedLeft, float speedRight) //in m/s
  {
    //and convert to ticks per frame
    speedLeft  *= (float)TICKS_PER_METER / (float)LOOP_RATE;
    speedRight *= (float)TICKS_PER_METER / (float)LOOP_RATE;

    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
    ivector speed(2); 
    speed[0] = speedLeft;
    speed[1] = speedRight;
        
    motionController.SetTarget(speed);
  }

  ivector CommandMotors(const ivector& effort) //actuators, generically
  {
    driver.SetPowers(effort[0], effort[1]);
    
    return effort;
  }

  Pose UpdatePrediction(int16_t deltaL, int16_t deltaR) //deltas in encoder ticks!!!
  /*
  Update the predicted pose. Basically, it calculates the forward movement as the
  average of the two delta motors. The angular velocity is the difference between
  the two.
  
  A better way would be to calculate wheel arcs, but that's for another day...
  */
  {
    //keep track of loop speed
    static unsigned long lastUpdate = millis();
    unsigned long currUpdate = millis();

    double dt = (currUpdate - lastUpdate) / 1000.0;
    lastUpdate = currUpdate;

    //pre-calculate cos and sin to save time
    double cos_theta = cos(currPose.theta);
    double sin_theta = sin(currPose.theta);

    //convert to meters
    float distL = deltaL / (float)TICKS_PER_METER;
    float distR = deltaR / (float)TICKS_PER_METER;

    //update the kinematics
    double dist = 0.5 * (distL + distR);
  
    currPose.x += dist * cos_theta;
    currPose.y += dist * sin_theta;

    currPose.theta += (distR - distL) / (2 * RADIUS_ROBOT);
    while(currPose.theta > M_PI)
      currPose.theta -= 2 * M_PI;
    
    while(currPose.theta < -M_PI)
      currPose.theta += 2 * M_PI;

//        String message;
//        message += String(dt);
//        message += '\t';
//        message += String(distL);
//        message += '\t';
//        message += String(distR);
//        message += '\t';
//        DEBUG_SERIAL.print(message);
 
    //Extended Kalman filter    
    //Update P: P- = A * P- * A.MakeTranspose() + W * Q * Wt;

    //pre-compute some factors
    double a = -dist * sin_theta; 
    double b =  dist * cos_theta;

    //A * P * A'
//    dmatrix A(3,3);
//    A[0][0] = 1;
//    A[0][2] = a;
//    A[1][1] = 1;
//    A[1][2] = b;
//    A[2][2] = 1;
    
    //P = A * P * A.MakeTranspose();
    
    P[0][0] += a * P[2][0] + a * P[0][2] + a * a * P[2][2];
    P[0][1] += a * P[2][1] + b * P[0][2] + a * b * P[2][2]; 
    P[0][2] += a * P[2][2];
   
    P[1][0] += b * P[2][0] + a * P[1][2] + a * b * P[2][2];
    P[1][1] += b * P[2][1] + b * P[1][2] + b * b * P[2][2]; 
    P[1][2] += b * P[2][2];
   
    P[2][0] += a * P[2][2];
    P[2][1] += b * P[2][2];
   
    //noise matrix
    dmatrix WQWt(3,3);
    
/*
 * For some reason, when I include the error terms that I think are 'right', the
 * system blows up in certain circumstances; e.g., when the GPS jumps around and
 * the robot is navigating to a waypoint. The x,y jumps substantially, but worse
 * is that theta grows huge.
 
 * so...I've also addes a little bit of uncorrelated, "bouncing" noise 
*/
    //first part is error due to encoders
    WQWt[0][0] = VAR_VEL_ENC * cos_theta * cos_theta * dt * dt / 2.0;
    WQWt[0][1] = VAR_VEL_ENC * sin_theta * cos_theta * dt * dt / 2.0;
    WQWt[0][2] = 0;
    WQWt[1][0] = VAR_VEL_ENC * sin_theta * cos_theta * dt * dt / 2.0;
    WQWt[1][1] = VAR_VEL_ENC * sin_theta * sin_theta * dt * dt / 2.0;
    WQWt[1][2] = 0;
    WQWt[2][0] = 0;
    WQWt[2][1] = 0;
    WQWt[2][2] = VAR_VEL_ENC * dt * dt / (RADIUS_ROBOT * RADIUS_ROBOT * 2.0);

    P += WQWt;
    
    //bouncing error (e.g., side slip)
    /*
     * This shouldn't be needed, but need to test. It may be useful to have some "random bounce" error
     */
//    P[0][0] += VAR_VEL_ENC * dt * dt;
//    P[1][1] += VAR_VEL_ENC * dt * dt;
//    P[2][2] += VAR_VEL_ENC * dt * dt;
    
    return currPose;
  }
  
  Pose ApplyObservation(float xGPS, float yGPS)
  /*
  Uses current GPS reading to update the currPose.
  */
  {
    DEBUG_SERIAL.println("kalman");
    //K = P H' * (H P H' + V R V')^-1

    dmatrix denominator(2,2); // this will be pre-constructed as the inverse
    denominator[0][0] = P[1][1] + VAR_POS_GPS;
    denominator[0][1] = -P[0][1];
    denominator[1][0] = -P[1][0];
    denominator[1][1] = P[0][0] + VAR_POS_GPS;
    
    double determinant = denominator[0][0] * denominator[1][1] - denominator[0][1] * denominator[1][0];
    denominator /= determinant;

    dmatrix PHt(3,2);
    PHt[0][0] = P[0][0];
    PHt[0][1] = P[0][1];
    PHt[1][0] = P[1][0];
    PHt[1][1] = P[1][1];
    PHt[2][0] = P[2][0];
    PHt[2][1] = P[2][1];
    
    dmatrix K = PHt * denominator;
    
    //innovation = z - H x
    dvector innovation(2);
    innovation[0] = xGPS - currPose.x;
    innovation[1] = yGPS - currPose.y;

    //delta x
    dvector xDelta = K * innovation;

    currPose.x += xDelta[0];
    currPose.y += xDelta[1];
    currPose.theta += xDelta[2];

    //Update P: P -= K H P
    dmatrix H(2, 3);
    H[0][0] = 1;
    H[1][1] = 1;
    P -= K * H * P; //this was missing H before...no idea how it worked.
         
    return currPose;
  }

};

#endif	/* ROBOT_H */

