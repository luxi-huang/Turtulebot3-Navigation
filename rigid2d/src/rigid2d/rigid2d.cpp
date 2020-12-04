#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "cmath"
#include <cstdlib> //c standard library


namespace rigid2d {

  /***************** Vector2D *****************/
  Vector2D::Vector2D()
  {
    x = 0;
    y = 0;
    norm_x = 0;
    norm_y = 0;
    rigid2d::Vector2D::normalize();
  }

  Vector2D::Vector2D(double x_, double y_)
  {
    x = x_;
    y = y_;
    rigid2d::Vector2D::normalize();
  }

  void Vector2D::normalize()
  {
    if (x != 0)
      {
        norm_x = x / sqrt(pow(x, 2) + pow(y, 2));
      } else {
        norm_x = 0;
      }

      if (y != 0)
      {
        norm_y = y / sqrt(pow(x, 2) + pow(y, 2));
      } else {
        norm_y = 0;
      }
  }

  Vector2D & Vector2D::operator+=(const Vector2D & rhs)
  {
    x += rhs.x;
    y += rhs.y;
    Vector2D::normalize();
    return *this;
  }

  Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
  {
    lhs+=rhs;
    return lhs;
  }

  Vector2D & Vector2D::operator-=(const Vector2D & rhs)
  {
    x -= rhs.x;
    y -= rhs.y;
    Vector2D::normalize();
    
    return *this;
  }

  Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
  {
    lhs-=rhs;
    return lhs;
  }

  Vector2D & Vector2D::operator*=(const double & scalar)
  {
    x *= scalar;
    y *= scalar;
    Vector2D::normalize();
    
    return *this;
  }

  Vector2D operator*(Vector2D v, const double & scalar)
  {
    // alternate definition (left multiply)
    // call operator*=() member function of lhs object (just above)
    v*=scalar;
    return v;
  }

  Vector2D operator*(const double & scalar, rigid2d::Vector2D v)
  {
    // alternate definition (right multiply)
    // call operator*=() member function of lhs object (just above)
    v*=scalar;
    return v;
  }

  double Vector2D::length(const rigid2d::Vector2D & v)
  {
    return sqrt(pow(v.x, 2) + pow(v.y, 2));
  }

  double Vector2D::distance(const Vector2D & v1, const Vector2D & v2)
  {
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
  }

  double Vector2D::angle(const Vector2D & v)
  {
    return atan2(v.y, v.x);
  }

  std::ostream & operator<<(std::ostream & os, const Vector2D & v)
  {
    os << "< " << v.x << " , " << v.y << " >" << "\n";
    return os;
  }

  std::istream & operator>>(std::istream & is, Vector2D & v)
  {
    std::cout << "Enter x:" << std::endl;
    is >> v.x;

    std::cout << "Enter y:" << std::endl;
    is >> v.y;

    return is;
  }

  /***************** Twist2D *****************/

  Twist2D::Twist2D()
  {
    theta_dot = 0.0;
    vx = 0.0;
    vy = 0.0;
  }

  Twist2D::Twist2D(double init_theta_dot, double init_vx, double init_vy)
  {
    theta_dot = init_theta_dot;
    vx = init_vx;
    vy = init_vy;
  }

  std::ostream & operator<<(std::ostream & os, const Twist2D & twist)
  {
    os << "degree_dt" << twist.theta_dot << " vx " << twist.vx << "vy "<< twist.vy <<"\n";
    return os;
  }

  std::istream & operator>>(std::istream & is, Twist2D & twist)
  {
    std::cout << "Enter twist.theta_dot:" << std::endl;
    is >> twist.theta_dot;

    std::cout << "twist.vx" << std::endl;
    is >> twist.vx;

    std::cout << "twist.vy" << std::endl;
    is >> twist.vy;

    return is;
  }

  /********** Transform2D() ***********/

  Transform2D::Transform2D()
  {
    trans_.x = 0.0;
    trans_.y = 0.0; 
    radians_ = 0.0;
  }

  Transform2D::Transform2D(const Vector2D & trans)
  {
    trans_.x = trans.x;
    trans_.y = trans.y;
    radians_ = 0.0;
  }

  Transform2D::Transform2D(double radians)
  {
    trans_.x = 0.0;
    trans_.y = 0.0;
    radians_ = radians;
  }

  Transform2D::Transform2D(const Vector2D & trans, double radians)
  {
    trans_.x = trans.x;
    trans_.y = trans.y;
    radians_ = radians;
  }

  Vector2D Transform2D::operator()(Vector2D v) const
  {
    Vector2D result;
    result.x = v.x * std::cos(radians_) - v.y * std::sin(radians_) + trans_.x;
    result.y = v.x * std::sin(radians_) + v.y * std::cos(radians_) + trans_.y;
    return result;
  }

  Transform2D Transform2D::inv() const
  {
    double inv_radians = -radians_;
    Vector2D inv_trans;
    inv_trans.x = -trans_.x * std::cos(radians_) - trans_.y * std::sin(this->radians_);
    inv_trans.y = trans_.x * std::sin(radians_) - trans_.y * std::cos(this->radians_);
    Transform2D inv_result(inv_trans, inv_radians);
    return inv_result;
  }

  Transform2D& Transform2D::operator*=(const Transform2D & rhs)
  {
    this->trans_.x = rhs.trans_.x * std::cos(this->radians_) - rhs.trans_.y * std::sin(this->radians_) + this->trans_.x;
    this->trans_.y = rhs.trans_.x * std::sin(this->radians_) + rhs.trans_.y * std::cos(this->radians_) + this->trans_.y;
    this->radians_ = normalize_angle(this->radians_ + rhs.radians_);
    return *this;
  }

  std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
  {
    os << "degrees:" << tf.radians_ << " "
       << "dx:" << tf.trans_.x << " "
       << "dy:" << tf.trans_.y << " " << std::endl;
    return os;
  }

  std::istream & operator>>(std::istream & is, Transform2D & tf)
  {
    Vector2D v;
    double degree, radians;
    std::cout << "Enter tf.x:" << std::endl;
    is >> v.x;
    std::cout << "Enter tf.y:" << std::endl;
    is >> v.y;
    std::cout << "Enter tf.theta:" << std::endl;
    is >> degree;

    radians = deg2rad(degree);
    Transform2D tf_new(v, radians);
    tf = tf_new;
    return is;
  }

  Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
  {
    lhs*=rhs;
    // A = lhs;
    return lhs;
  }

  Twist2D Transform2D::operator()(Twist2D t) const
  {
    Twist2D result;
    result.theta_dot = t.theta_dot;
    result.vx = this->trans_.y * t.theta_dot + std::cos(this->radians_) * t.vx - std::sin(this->radians_) * t.vy;
    result.vy = -this->trans_.x * t.theta_dot + std::sin(this->radians_) * t.vx + std::cos(this->radians_) * t.vy;
    return result;
  }

  Transform2D displacement(const Transform2D & T)
  {
    Transform2D t;
    t = T;
    return t;
  }

  Transform2D integrateTwist(Twist2D twist)
  {
    if (almost_equal(twist.theta_dot, 0.0))
    {
      Vector2D result_vector(twist.vx, twist.vy);
      Transform2D result_transform(result_vector, 0);
      return result_transform;
    }
    else
    {
      double result_rad = twist.theta_dot;
      Vector2D result_vector;
      
      twist.vx = twist.vx / twist.theta_dot;
      twist.vy = twist.vy / twist.theta_dot;
      result_vector.x = std::sin(twist.theta_dot) * twist.vx + (std::cos(twist.theta_dot) - 1) * twist.vy;
      result_vector.y = (1 - std::cos(twist.theta_dot)) * twist.vx + std::sin(twist.theta_dot) * twist.vy;

      Transform2D result_transform(result_vector, result_rad);
      return result_transform;
    }
  }

}
