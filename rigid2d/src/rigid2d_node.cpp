#include "rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library

// almost_equal(0.0, 0.0);

using namespace rigid2d;

int main(){
  struct Vector2D v_ab, v_bc, v, va,vb,vc;
  double degree_ab,degree_bc, radians_ab, radians_bc;
  Transform2D Tba,Tcb,I, Tac, Tca;
  char frame;
  struct Twist2D twist1, twist_a,twist_b,twist_c;
  // Transform2D I;
  // Transform2D I;
  // struct Vector2D m;
  // v.x = 3.3;
  // v.y = 2.2;

  I = Transform2D();
  std::cout << "I : "<<I<<"\n";

  std::cout << "Enter x value of Tab\n";
  std::cin >> v_ab.x;
  std::cout << "Enter y value of Tab\n";
  std::cin >> v_ab.y;
  std::cout << "Enter degree value of Tab\n";
  std::cin >> degree_ab;
  radians_ab = deg2rad(degree_ab);
  Transform2D Tab(v_ab, radians_ab);
  std::cout << "Tab : "<<Tab<<"\n";

  std::cout << "Enter x value of Tbc\n";
  std::cin >> v_bc.x;
  std::cout << "Enter y value of Tbc\n";
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
  std::cout << "Tca : "<<Tca<<"\n";

  // enter vector v :
  std::cout << "please enter vector v.x\n";
  std::cin >> v.x;
  std::cout << "please enter vector v.y\n";
  std::cin >> v.y;
  // enter frame;
  std::cout << "please enter the frame: a,b,c\n";
  std::cin >> frame;

  //enter twist1;

  std::cout << "please enter twist:theta_dot\n";
  std::cin >> twist1.theta_dot;
  std::cout << "please enter vector vx\n";
  std::cin >> twist1.vx;
  std::cout << "please enter vector vy\n";
  std::cin >> twist1.vy;

  // output vector v in frame a,b and c;
  if (frame == 'a'){
    va = v;
    std::cout << "va : "<<va.x<<","<<va.y<<"\n";
    vb = Tba(v);
    std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
    vc = Tca(v);
    std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

    twist_a = I(twist1);
    twist_b = Tba(twist1);
    twist_c = Tca(twist1);

  }else if (frame == 'b'){
    va = Tab(v);
    std::cout << "va : "<<va.x<<","<<va.y<<"\n";
    vb = v;
    std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
    vc = Tcb(v);
    std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

    twist_a = Tab(twist1);
    twist_b = I(twist1);
    twist_c = Tcb(twist1);
  }else if (frame == 'c'){
    va = Tac(v);
    std::cout << "va : "<<va.x<<","<<va.y<<"\n";
    vb = Tbc(v);
    std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
    vc = v;
    std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

    twist_a = Tca(twist1);
    twist_b = Tcb(twist1);
    twist_c = I(twist1);
  }

  std::cout << "twist in franme a : theta_dot: "<<twist_a.theta_dot<<", vx: "<<twist_a.vx<<", vy"<< twist_a.vy<<"\n";
  std::cout << "twist in franme b : theta_dot: "<<twist_b.theta_dot<<", vx: "<<twist_b.vx<<", vy"<< twist_b.vy<<"\n";
  std::cout << "twist in franme c : theta_dot: "<<twist_c.theta_dot<<", vx: "<<twist_c.vx<<", vy"<< twist_c.vy<<"\n";

  return 0;
}
