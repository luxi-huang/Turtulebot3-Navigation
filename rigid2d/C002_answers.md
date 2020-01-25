# Homework0 C002 Answer

### Question1 : the difference between class and structure:
*  The only difference between a struct and class in C++ is the default accessibility of member variables and methods. In a struct they are public; in a class they are private.

### Question2 : Why is Vector2D a struct and Transform2DClass?
* From the classes and calss hierachies:
  * since Trasnform2DClasses has invariant, the data member can vary independently.
  * In my Transform2D class, some variables are priviate, but all vairables in struct are public.

### Question3:
* C.21: If you define or =delete any default operation, define or =delete them all
* C.43: Ensure that a copyable (value type) class has a default constructor
