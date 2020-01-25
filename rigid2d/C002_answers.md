# Homework0 C002 Answer

### Question1 : the difference between class and structure:
*  The only difference between a struct and class in C++ is the default accessibility of member variables and methods. In a struct they are public; in a class they are private.

### Question2 : Why is Vector2D a struct and Transform2DClass?
* From the classes and calss hierachies:
  - C30:Define a destructor if a class needs an explicit action at object destruction.
  - C32 : A base class destructor should be either public and virtual, or protected and nonvirtual.       
* since Trasnform2DClasses has invariant, the data member can vary independently.In my Transform2D class, some variables are priviate, but all vairables in struct are public.

### Question 3:
* C.21: If you define or =delete any default operation, define or =delete them all
* C.43: Ensure that a copyable (value type) class has a default constructor

### Question 4:
1.  Vector2D operator()(Vector2D v) const;
   * this function apply the operator, v is constant, and return type is vector 2D;  
2.  Vector2D normalize()(Vector2D v) const;
   * This is a function, v is a constant, and return type is Vector2D.
3.  Vector2D & operator*=(const Vector2D & v);
  * This operator return the point of Vector2D.

### Question 5
I would like to implement:
```
Vector2D normalize()(Vector2D v) const{
  Vector2D normalize_vector;
  double length square_root ;
  length = std::pow(v.x,2) + std::pow(v.y,2);
  square_root = std::sqrt(lengh);
  normalize_vector.x = v.x / sqrt_root;
  normalize_vector.y = v.y / sqrt_root;
  return normalize_vector;
}
```
### Question 6
Con.4: Use const to define objects with values that do not change after construction
