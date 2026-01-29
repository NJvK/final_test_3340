#include "../include/pros/distance_util.hpp"

// define constants here or in the header
static constexpr double FRONT_OFFSET = 0;
static constexpr double BACK_OFFSET  = 5;
static constexpr double LEFT_OFFSET  = 5;
static constexpr double RIGHT_OFFSET = 5;
static constexpr double HALF_FIELD   = 72.0; // example in inches, you set this

DistanceUtil::DistanceUtil()
  : distancef(2), distanceb(10), distancel(4), distancer(9) {}

double DistanceUtil::mmToIn(double mm) {
  return mm / 25.4;
}

double DistanceUtil::safeRead(pros::Distance& sensor) {
  int mm = sensor.get();
  if (mm < 0) return -1;
  if (mm == 9999) return -1;
  return mmToIn(mm);
}

void DistanceUtil::resetcoord(int quadrant, int angle, lemlib::Chassis& chassis) {
  double frontRaw = safeRead(distancef);
  double backRaw  = safeRead(distanceb);
  double leftRaw  = safeRead(distancel);
  double rightRaw = safeRead(distancer);

  lemlib::Pose current = chassis.getPose();
  double xPos = current.x;
  double yPos = current.y;

  double front = (frontRaw >= 0) ? (frontRaw + FRONT_OFFSET) : -1;
  double back  = (backRaw  >= 0) ? (backRaw  + BACK_OFFSET)  : -1;
  double left  = (leftRaw  >= 0) ? (leftRaw  + LEFT_OFFSET)  : -1;
  double right = (rightRaw >= 0) ? (rightRaw + RIGHT_OFFSET) : -1;
  
  bool red = false;
    bool blue = false;
    bool leftd = false;
    bool rightd = false;

    switch (angle)
    {
    case 0:
        blue = true;
        break;
    case 90:
        rightd = true;
        break;
    case 180:
        red = true;
        break;
    case 270:
        leftd = true;
        break;
    default:
        break;
    }

    // QUAD 1

    if (quadrant == 1){
        if(red){
            xPos = (HALF_FIELD - left);
            yPos = (HALF_FIELD - back);

        }
        else if (blue){
            xPos = (HALF_FIELD - right);
            yPos = (HALF_FIELD - front);
        }
        else if (rightd){
            xPos = (HALF_FIELD - front);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd){
            xPos = (HALF_FIELD - back);
            yPos = (HALF_FIELD - right);
        }
    
    }

    // QUAD 2

    if (quadrant == 2){
        if(red){
            xPos = -(HALF_FIELD - right);
            yPos = (HALF_FIELD - back);
        }
        else if (blue){
            xPos = -(HALF_FIELD - left);
            yPos = (HALF_FIELD - front);
        }
        else if (rightd){
            xPos = -(HALF_FIELD - back);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd){
            xPos = -(HALF_FIELD - front);
            yPos = (HALF_FIELD - right);
        }
    
    }

    // QUAD 3

    if (quadrant == 3){
        if(red){
            xPos = -(HALF_FIELD - right);
            yPos = -(HALF_FIELD - front);
        }
        else if (blue){
            xPos = -(HALF_FIELD - left);
            yPos = -(HALF_FIELD - back);
        }
        else if (rightd){
            xPos = -(HALF_FIELD - back);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd){
            xPos = -(HALF_FIELD - front);
            yPos = -(HALF_FIELD - left);
        }
    
    }

    // QUAD 4

    if (quadrant == 4){
        if(red){
            xPos = (HALF_FIELD - left);
            yPos = -(HALF_FIELD - front);

        }
        else if (blue){
            xPos = (HALF_FIELD - right);
            yPos = -(HALF_FIELD - back);
        }
        else if (rightd){
            xPos = (HALF_FIELD - front);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd){
            xPos = (HALF_FIELD - back);
            yPos = -(HALF_FIELD - left);
        }
    
    }

  // your quadrant/angle logic here...
  chassis.setPose(xPos, yPos, chassis.getPose().theta);
}