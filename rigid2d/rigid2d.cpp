#include "rigid2d.hpp"
#include <iostream>
#include "math.h"

namespace rigid2d{

  double pi = rigid2d::PI ;

  // bool rigid2d::almost_equal(double d1, double d2, double epsilon=1.0e-12){
  //   if(abs(d1 - d2) < epsilon){
  //     return true;
  //   }else{
  //     return false;
  //   }
  // }

  // double rigid2d::deg2rad(double deg){
  //   return deg / 180 * PI;
  // }
  //
  double rigid2d::rad2deg(double rad){
    return 180 * rad / pi;
  }

}
