#include "rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

// almost_equal(0.0, 0.0);

using namespace rigid2d;

int main(){
  struct Vector2D v_ab, v_bc v;
  double degree_ab,degree_bc, radians_ab, radians_bc;
  Transform2D Tba,Tcb,I, Tac;
  // Transform2D I;
  // Transform2D I;
  // struct Vector2D m;
  // v.x = 3.3;
  // v.y = 2.2;

  I = Transform2D();
  std::cout << "I : "<<I<<"\n";

  std::cout << "Enter x value of Tab \n";
  std::cin >> v_ab.x;
  std::cout << "Enter y value of Tab \n";
  std::cin >> v_ab.y;
  std::cout << "Enter degree value of Tab\n";
  std::cin >> degree_ab;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);
  std::cout << "Tab : "<<Tab<<"\n";

  std::cout << "Enter x value of Tbc \n";
  std::cin >> v_bc.x;
  std::cout << "Enter y value of Tbc \n";
  std::cin >> v_bc.y;
  std::cout << "Enter degree value of Tbc\n";
  std::cin >> degree_bc;
  radians_bc = deg2rad(degree_bc);
  Transform2D Tbc(v_bc, radians_bc);
  std::cout << "Tbc : "<<Tbc<<"\n";


  Tab *= I;
  std::cout << "Tab : "<<Tab<<"\n";
  Tba = Tab.inv();
  std::cout << "Tba : "<<Tba<<"\n";

  Tbc *=I;
  std::cout << "Tbc : "<<Tbc<<"\n";
  Tcb = Tbc.inv();
  std::cout << "Tba : "<<Tcb<<"\n";

  Tac = Tab*Tbc;
  std::cout << "Tac : "<<Tac<<"\n";

  Tca = Tac.inv();
  std::cout << "Tba : "<<Tca<<"\n";




  // std::cout << v.y <<"\n";
  // Transform2D T(30); //rotate
  // m = T(v);
  // Transform2D P(30);
  // std::cin>>P;
  // Transform2D P(30); // identity matrix
  // T = Transform2D();
  // v = P(v);
  // P.inv();
  // std::cout << v.x;
  // P = P.inv();
  // m = P(v);
  // std::cout << m.x;
  return 0;
}
