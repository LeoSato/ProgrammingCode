//main.cpp
#include "shape.h"
#include <iostream>
//#include "gnuplot-iostream.h"

using namespace std;

int main(){
  float center1[2]={1.0,1.0};
  float center2[2]={2.0,2.0};
  Rectangle r(2.0,3.0);
  Triangle t(5.0);
  Square s;
  Circle c(1.0,center1);
  cout<<"Rectangle Area: "<<r.getArea()<<endl;
  cout<<"Rectangle Perimeter: "<<r.getPerimeter()<<endl;
  cout<<"Triangle Area: "<<t.getArea()<<endl;
  cout<<"Triangle Perimeter: "<<t.getPerimeter()<<endl;
  cout<<"Square Area: "<<s.getArea()<<endl;
  cout<<"Square Perimeter: "<<s.getPerimeter()<<endl;
  cout<<"Circle Area: "<<c.getArea()<<endl;
  cout<<"Circle Circumference: "<<c.getPerimeter()<<endl;
  float *c_alias=c.getCenter();
  cout<<"Circle Center: ("<<*c_alias<<","<<*(c_alias+1)<<")"<<endl;
  c.setCenter(center2);
  c_alias=c.getCenter();
  cout<<"Circle New Center: ("<<*c_alias<<","<<*(c_alias+1)<<")"<<endl;
  cout<<"Center1 array: ("<<center1[0]<<","<<center1[1]<<")"<<endl;
  cout<<"Center2 array: ("<<center2[0]<<","<<center2[1]<<")"<<endl;
  return 0;
}
