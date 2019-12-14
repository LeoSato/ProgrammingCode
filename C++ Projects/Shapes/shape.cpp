//shape.cpp
#include "shape.h"
#include <cmath>
#define PI 3.14159265

float * Shape::getCenter(){
  static float * c = center;
  return c;
}
void Shape::setCenter(float (&c)[2]){
  *center = *c;
  *(center+1) = *(c+1);
}
Rectangle::Rectangle(float w, float h){
  width = w;
  height = h;
}
float Rectangle::getArea(){
  return width * height;
}
float Rectangle::getPerimeter(){
  return 2 * (width + height);
}
Triangle::Triangle(float b, float h){
  base = b;
  height = h;
}
float Triangle::getArea(){
  return 0.5 * base * height;
}
float Triangle::getPerimeter(){
  return base + 2 * pow(base * base + height * height,0.5);
}
Circle::Circle(float r){
  radius = r;
}
Circle::Circle(float r, float (&c)[2]){
  radius = r;
  Shape::setCenter(c);
}
float Circle::getArea(){
  return PI * pow(radius,2.0);
}
float Circle::getPerimeter(){
  return PI * 2 * radius;
}
