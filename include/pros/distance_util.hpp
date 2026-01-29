#pragma once

#include "main.h"
#include "lemlib/api.hpp"

class DistanceUtil {
public:
  DistanceUtil();

  static double mmToIn(double mm);
  static double safeRead(pros::Distance& sensor);

  void resetcoord(int quadrant, int angle, lemlib::Chassis& chassis);

private:
  pros::Distance distancef;
  pros::Distance distanceb;
  pros::Distance distancel;
  pros::Distance distancer;
};
