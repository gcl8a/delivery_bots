#ifndef __POSE_H
#define __POSE_H

struct Pose
{
  double x;
  double y;
  double theta;
  
  unsigned long timestamp;
  
  Pose(void) {x=y=theta=0; timestamp = 0;}
  Pose(double _x, double _y, double _th)
  {
    x = _x;
    y = _y;
    theta = _th;

    timestamp = millis();
  }
};

float CalcDistanceBetweenPoses(const Pose& a, const Pose& b)
{
  float sqd = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
  return sqrt(sqd);
}

float CalcAngleToPose(const Pose& b, const Pose& a)
//angle from a to b; b comes first
{
  float ang = atan2(b.y - a.y, b.x - a.x);
  return ang;
}

#endif
