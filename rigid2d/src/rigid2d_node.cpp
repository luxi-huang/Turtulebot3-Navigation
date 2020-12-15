#include <cstdlib>
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include <fstream>

using namespace rigid2d;
using namespace std;

int main(){
  /* Initial variables */ 
  Transform2D Tab, Tbc, Tba,Tcb,I, Tac, Tca;
  char frame;
  struct Twist2D twist1, twist_a,twist_b,twist_c;

  /* Ask user to enter two transforms Tab and Tbc */ 
  std::cout << "Enter Tab" << "\n";
  std::cin >> Tab;
  cout << "Tab: " << Tab << "\n";

  std::cout << "Enter Tbc" << "\n";
  std::cin >> Tab;
  cout << "Tbc: " << Tab << "\n";

  /* calculate inverse Tba and Tcb*/
  Tba = Tab.inv();
  std::cout << "Tba : "<<Tba<<"\n";

  Tcb = Tbc.inv();
  std::cout << "Tcb : "<<Tcb<<"\n";

  /* calculate Tac */
  Tac = Tab*Tbc;
  std::cout << "Tac : "<<Tac<<"\n";

  /*calculate Tca */
  Tca = Tac.inv();
  std::cout << "Tca : "<<Tca<<"\n";

  /* Enter Twist */
  std::cout << "please enter twist \n";
  std::cin >> twist1;
  std::cout << "twist : "<<twist1<<"\n";

  //* Enter Frame
  std::cout << "please enter the frame: a,b,c\n";
  std::cin >> frame;

  cout << "cos_30" << cos(30) <<"\n";
  
}

//   // enter vector v :
//   std::cout << "please enter vector v.x\n";
//   std::cin >> v.x;
//   std::cout << "please enter vector v.y\n";
//   std::cin >> v.y;


//   // output vector v in frame a,b and c;
//   if (frame == 'a'){
//     va = v;
//     std::cout << "va : "<<va.x<<","<<va.y<<"\n";
//     vb = Tba(v);
//     std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
//     vc = Tca(v);
//     std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

//     twist_a = I(twist1);
//     twist_b = Tba(twist1);
//     twist_c = Tca(twist1);

//   }else if (frame == 'b'){
//     va = Tab(v);
//     std::cout << "va : "<<va.x<<","<<va.y<<"\n";
//     vb = v;
//     std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
//     vc = Tcb(v);
//     std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

//     twist_a = Tab(twist1);
//     twist_b = I(twist1);
//     twist_c = Tcb(twist1);
//   }else if (frame == 'c'){
//     va = Tac(v);
//     std::cout << "va : "<<va.x<<","<<va.y<<"\n";
//     vb = Tbc(v);
//     std::cout << "vb : "<<vb.x<<","<<vb.y<<"\n" ;
//     vc = v;
//     std::cout << "vc : "<<vc.x<<","<<vc.y<<"\n";

//     twist_a = Tca(twist1);
//     twist_b = Tcb(twist1);
//     twist_c = I(twist1);
//   }

//   std::cout << "twist in franme a : theta_dot: "<<twist_a.theta_dot<<", vx: "<<twist_a.vx<<", vy"<< twist_a.vy<<"\n";
//   std::cout << "twist in franme b : theta_dot: "<<twist_b.theta_dot<<", vx: "<<twist_b.vx<<", vy"<< twist_b.vy<<"\n";
//   std::cout << "twist in franme c : theta_dot: "<<twist_c.theta_dot<<", vx: "<<twist_c.vx<<", vy"<< twist_c.vy<<"\n";

//   return 0;
// }
