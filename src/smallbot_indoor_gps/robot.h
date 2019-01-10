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

#define VAR_BIAS    0   //1e-5
#define VAR_VEL_ENC 1e-1
#define VAR_POS_GPS 1e-0
#define PIXELS_PER_METER 200.0

#define DESTINATION_TOLERANCE 0.1  //m

 /*
 * Calculation for RADIANS_PER_TICK:
 * 12 magnets = 48 ticks / rotation
 * Gear ratio of 9.7:1 -> 465.6 ticks / wheel rotation 
 * = 465.6 ticks / rot / (0.285 m / rot)
 */

#define TICKS_PER_METER 364
#define RADIUS_ROBOT 0.047

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

  float biasL = 0;
  float biasR = 0;
  
  TMatrix<float> P;
  TMatrix<float> Q;

  double K_dist = 0.5;
  double K_ang  = 2.0;

  volatile uint8_t readyToReport = 0; //does this need to be volatile?

public:
  UGV(void) : wheel_speeds(2), effort(2), P(5,5), Q(4,4)
  {
    Q[0][0] = VAR_VEL_ENC;
    Q[1][1] = VAR_VEL_ENC;

    Q[2][2] = VAR_BIAS;
    Q[3][3] = VAR_BIAS;
  }
  
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
      readyToReport = 1;
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
      DEBUG_SERIAL.print(millis());
      DEBUG_SERIAL.print(": ");
      
      wheel_speeds = motionController.CalcEstimate(); //wheel velocity is ticks/period

      DEBUG_SERIAL.print("[w: ");
      DEBUG_SERIAL.print(wheel_speeds[0]);
      DEBUG_SERIAL.print(' ');
      DEBUG_SERIAL.print(wheel_speeds[1]);
      DEBUG_SERIAL.print("]\t");

      //get the latest prediction (stored in currPose):
      UpdatePredictionWithBias(wheel_speeds[0], wheel_speeds[1]);

      if(robotState == DRIVE_DEAD_RECK)
      {
        dvector error(2);
        error[0] = CalcDistanceBetweenPoses(destPose, currPose);

        float angleError = CalcAngleToPose(destPose, currPose) - currPose.theta;
        while(angleError >  M_PI) {angleError -= 2 * M_PI;}
        while(angleError < -M_PI) {angleError += 2 * M_PI;}
        error[1] = angleError;
        
        DEBUG_SERIAL.print("[e: ");
        DEBUG_SERIAL.print(error[0]);
        DEBUG_SERIAL.print(' ');
        DEBUG_SERIAL.print(error[1]);
        DEBUG_SERIAL.print("]\t");

        SetTwistSpeed(error[0] * K_dist, error[1] * K_ang);
  
        effort = motionController.CalcEffort();
        
        DEBUG_SERIAL.print("[f: ");
        DEBUG_SERIAL.print(effort[0]);
        DEBUG_SERIAL.print(' ');
        DEBUG_SERIAL.print(effort[1]);
        DEBUG_SERIAL.print("]\t");

        CommandMotors(effort);
      }
  }
  
  void SetTwistSpeed(float vel, float ang_vel)
  {
    if(fabs(vel) > 0.5) vel *= 0.5 / fabs(vel);
    if(fabs(ang_vel) > 4.0) ang_vel *= 4.0 / fabs(ang_vel);

    //do calculations in m/s
    float speedLeft  = vel - ang_vel * RADIUS_ROBOT;
    float speedRight = vel + ang_vel * RADIUS_ROBOT;

    DEBUG_SERIAL.print("[t: ");
    DEBUG_SERIAL.print(speedLeft);
    DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(speedRight);
    DEBUG_SERIAL.print("]\t");

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

    DEBUG_SERIAL.print("[t: ");
    DEBUG_SERIAL.print(speed[0]);
    DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(speed[1]);
    DEBUG_SERIAL.print("]\t");
        
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
    DEBUG_SERIAL.println("You shouldn't be here!!!");
    //keep track of loop speed
    static unsigned long lastUpdate = millis();
    unsigned long currUpdate = millis();

    double dt = (currUpdate - lastUpdate) / 1000.0;
    lastUpdate = currUpdate;

    //pre-calculate cos and sin to save time
    float cos_theta = cos(currPose.theta);
    float sin_theta = sin(currPose.theta);

    //convert to meters
    float distL = deltaL / (float)TICKS_PER_METER;
    float distR = deltaR / (float)TICKS_PER_METER;

    //update the kinematics
    float dist = 0.5 * (distL + distR);
  
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
    float a = -dist * sin_theta; 
    float b =  dist * cos_theta;

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
    TMatrix<float> WQWt(3,3);

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
    
    /*
     * It is be useful to have some "random bounce" error
     */
//    P[0][0] += VAR_VEL_ENC * dt * dt;
//    P[1][1] += VAR_VEL_ENC * dt * dt;
//    P[2][2] += VAR_VEL_ENC * dt * dt;
    
    return currPose;
  }
  
  Pose ApplyObservation(float uGPS, float vGPS) //in pixel space!
  /*
  * Uses current GPS reading to update the currPose. "GPS" coordinates are in pixels, where the u,v axes are
  * inverted from x.y space.
  * 
  * In tests, this function takes < 2ms, which is nothing compared to the delay in the camera.
  */
  {
    //DEBUG_SERIAL.println("Correcting");
    //K = P H' * (H P H' + V R V')^-1 [n.b., V = I]

    TMatrix<float> H(2, 5);

    float gamma2 = PIXELS_PER_METER * PIXELS_PER_METER;
    H[0][1] = PIXELS_PER_METER;
    H[1][0] = PIXELS_PER_METER;

    TMatrix<float> denominator(2,2); // this will be pre-constructed as the inverse
    denominator[0][0] =  gamma2 * P[0][0] + VAR_POS_GPS;
    denominator[0][1] = -gamma2 * P[1][0];
    denominator[1][0] = -gamma2 * P[0][1];
    denominator[1][1] =  gamma2 * P[1][1] + VAR_POS_GPS;
    
    float determinant = denominator[0][0] * denominator[1][1] - denominator[0][1] * denominator[1][0];
    denominator /= determinant;

    TMatrix<float> PHt(5,2);
    PHt[0][0] = PIXELS_PER_METER * P[0][1];
    PHt[0][1] = PIXELS_PER_METER * P[0][0];
    PHt[1][0] = PIXELS_PER_METER * P[1][1];
    PHt[1][1] = PIXELS_PER_METER * P[1][0];
    PHt[2][0] = PIXELS_PER_METER * P[2][1];
    PHt[2][1] = PIXELS_PER_METER * P[2][0];
    PHt[3][0] = PIXELS_PER_METER * P[3][0];
    PHt[3][1] = PIXELS_PER_METER * P[3][1];
    PHt[4][0] = PIXELS_PER_METER * P[4][0];
    PHt[4][1] = PIXELS_PER_METER * P[4][1];
    
    TMatrix<float> K = PHt * denominator;

    //innovation = z - H x
    fvector z(2);
    z[0] = uGPS;
    z[1] = vGPS;
    
    fvector Hx(2);
    Hx[0] = PIXELS_PER_METER * currPose.y;
    Hx[1] = PIXELS_PER_METER * currPose.x;
//    x[2] = currPose.theta;
//    x[3] = biasL;
//    x[4] = biasR;

    TVector<float> innovation = z - Hx;

    //delta x
    TVector<float> xDelta = K * innovation;

    currPose.x += xDelta[0];
    currPose.y += xDelta[1];
    currPose.theta += xDelta[2];
    biasL += xDelta[3];
    biasR += xDelta[4];

    //Update P: P -= K H P
    P -= K * H * P; //H has lots of zeros; could be made more efficient

//    for(int i = 0; i < 5; i++)
//    {
//      for(int j = 0; j < 5; j++)
//      {
//        DEBUG_SERIAL.print(P[i][j]);
//        DEBUG_SERIAL.print('\t');
//      }
//      
//      DEBUG_SERIAL.print('\n');
//    }

    return currPose;
  }

  Pose UpdatePredictionWithBias(int16_t deltaL, int16_t deltaR) //deltaN is in encoder ticks!!!
  /*
  Update the predicted pose. Basically, it calculates the forward movement as the
  average of the two delta motors. The angular velocity is the difference between
  the two.

  This takes ~4ms to complete, but I could save some time by multiplying matrices better (A is upper diagonal)
  or -- better yet -- precomputing everything...

  Now would be a good time to move to a uC with an FPU...

  In tests, this routine takes < 3ms
  */
  {
    //keep track of loop speed
    static unsigned long lastUpdate = millis();
    unsigned long currUpdate = millis();

    double dt = (currUpdate - lastUpdate) / 1000.0;
    lastUpdate = currUpdate;

    //pre-calculate cos and sin to save time
    float cos_theta = cos(currPose.theta);
    float sin_theta = sin(currPose.theta);

    //convert to meters
    float distL = deltaL / (float)TICKS_PER_METER - biasL;
    float distR = deltaR / (float)TICKS_PER_METER - biasR;

    //update the kinematics
    float dist = 0.5 * (distL + distR);
  
    currPose.x += dist * cos_theta;
    currPose.y += dist * sin_theta;

    currPose.theta += (distR - distL) / (2 * RADIUS_ROBOT);
    while(currPose.theta > M_PI)
      currPose.theta -= 2 * M_PI;
    
    while(currPose.theta < -M_PI)
      currPose.theta += 2 * M_PI;
 
    //Extended Kalman filter    
    //Update P: P- = A * P * A' + W * Q * W';

    //pre-compute some factors
    float a = dist * sin_theta; 
    float b = dist * cos_theta;

    TMatrix<float> A(5,5);
    A[0][0] = 1.0;
    A[0][2] = -a;
    A[0][3] = -dt * cos_theta / 2.0;
    A[0][4] = A[0][3];

    A[1][1] = 1.0;
    A[1][2] = b;
    A[1][3] = -dt * sin_theta / 2.0;
    A[1][4] = A[1][3];

    A[2][2] = 1.0;
    A[2][3] = dt / (2 * RADIUS_ROBOT);
    A[2][4] = A[2][3];

    A[3][3] = 1.0;
    
    A[4][4] = 1.0;

    /*
     * A is upper triangular, so we can save over half of the multiplies if we skip the generic matrix multiplier.
     * Faster still would be to work through the maths, but this is just a test, and this function will come in under 2 ms.
     */
    TMatrix<float> AP(5,5);
    for(int i = 0; i < 5; i++)
    {
      for(int j = 0; j < 5; j++)
      {
        AP[i][j] = P[i][j];
        for(int k = i+1; k < 5; k++)  
        {
          AP[i][j] += A[i][k] * P[k][j];
        }
      }
    }

    TMatrix<float> APAt(5,5);
    for(int i = 0; i < 5; i++)
    {
      for(int j = 0; j < 5; j++)
      {
        APAt[i][j] = AP[i][j];
        for(int k = j+1; k < 5; k++)  
        {
          APAt[i][j] += AP[i][k] * A[j][k];
        }
      }
    }

    TMatrix<float> W(5,4);
    W[0][0] = -A[0][3];
    W[0][1] =  W[0][0];
    W[0][2] = -W[0][0];
    W[0][3] =  W[0][2];

    W[1][0] = -A[1][3];
    W[1][1] =  W[1][0];
    W[1][2] = -W[1][0];
    W[1][3] =  W[1][2];

    W[2][0] = A[2][4];
    W[2][1] = A[2][3];
    W[2][2] = A[2][3];
    W[2][3] = A[2][4];

    W[3][2] = 1.0;
    
    W[4][3] = 1.0;

    //noise matrix
    /*
     * WQW' can be done a bit faster (roughly 2/3 the base time) by taking advantage of the diagonality of Q.
     * Still better would be to work through the maths, which aren't too hard, but this is good enough for now.
     */
    TMatrix<float> WQWt(5,5);

    for(int k = 0; k < 4; k++)
    {
      for(int i = 0; i < 5; i++)
      {
        float WQ = W[i][k] * Q[k][k];
        for(int j = 0; j < 5; j++)
        {
          WQWt[i][j] += WQ * W[j][k];
        }
      }
    }

    P = APAt + WQWt;
    /*
     * It can be useful to have some "random bounce" error:
     */
    //P[0][0] += VAR_VEL_ENC * dt * dt;
    //P[1][1] += VAR_VEL_ENC * dt * dt;
    //P[2][2] += VAR_VEL_ENC * dt * dt;
    
    return currPose;
  }
  
};

#endif	/* ROBOT_H */

