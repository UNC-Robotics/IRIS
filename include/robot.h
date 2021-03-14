#ifndef ROBOT_H
#define ROBOT_H

class Robot {
  public:
    Robot() = default;
    ~Robot() = default;

    virtual void Initialize() = 0;
    virtual void ComputeShape() = 0;

};

#endif // ROBOT_H