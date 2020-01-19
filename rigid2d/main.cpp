#include "rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

// almost_equal(0.0, 0.0);

using namespace rigid2d;

int main(){
  struct Vector2D v;
  v.x = 3.3;
  v.y = 2.2;
  std::cout << "x";
  std::cin >> v.x;
  std::cout << "y";
  std::cin >> v.y;

  std::cout << v.x << " ";
  std::cout << v.y <<"\n";
  return 0;
}
