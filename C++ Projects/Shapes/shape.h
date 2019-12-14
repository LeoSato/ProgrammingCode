//shape.h
#ifndef SHAPE_H
#define SHAPE_H

class Shape{
  public:
    virtual float getArea()=0;
    virtual float getPerimeter()=0;
    float * getCenter();
    void setCenter(float (&c)[2]);
  protected:
    float center[2];
};
class Rectangle: public Shape{
  public:
    Rectangle(float w = 1.0,float h = 1.0);
    float getArea();
    float getPerimeter();
  private:
    float width, height;
};
//Equilateral or isosceles Triangle
class Triangle: public Shape{
  public:
    Triangle(float b = 1.0, float h = 1.0);
    float getArea();
    float getPerimeter();
  private:
    float base, height;
};
class Square: public Rectangle{
  public:
    Square(float side = 1.0): Rectangle(side,side) {};
};
class Circle: public Shape{
  public:
    Circle(float r = 1.0);
    Circle(float r, float (&c)[2]);
    float getArea();
    float getPerimeter();
  private:
    float radius;
};
#endif
