#include <PID_v1.h>
#ifndef PIDControl_H
#define PIDControl_H
class BallControl {
public:
    BallControl *p_next;
    BallControl *p_prev;
    PID *myPID;
    Me7SegmentDisplay * myDisp;
    double Kp;
    double Ki;
    double Kd;
    BallControl(PID *pid, Me7SegmentDisplay* disp);
    virtual void display();
    virtual void increase(double delta);
    virtual void decrease(double delta);
};

class ProportionalControl: public BallControl {
public:
    ProportionalControl(PID *pid, Me7SegmentDisplay* disp);
    void display();
    void increase(double delta);
    void decrease(double delta);
};

class IntegralControl: public BallControl {
public:
    IntegralControl(PID *pid, Me7SegmentDisplay* disp);
    void display();
    void increase(double delta);
    void decrease(double delta);
};

class DerivitiveControl: public BallControl {
public:
    DerivitiveControl(PID *pid, Me7SegmentDisplay* disp);
    void display();
    void increase(double delta);
    void decrease(double delta);
};  

class PIDControl: public BallControl {
public:
    double * servoPosition;
    PIDControl(PID *pid, Me7SegmentDisplay* disp, double * position);
    void display();
    void increase(double delta);
    void decrease(double delta);
};  

class SetPointControl: public BallControl {
public:
    double * setPoint;
    SetPointControl(PID *pid, Me7SegmentDisplay* disp, double * p_setPoint);
    void display();
    void increase(double delta);
    void decrease(double delta);
};  

class MeasurementControl: public BallControl {
public:
    double * myMeasurement;
    MeasurementControl(PID *pid, Me7SegmentDisplay* disp, double * measurement);
    void display();
    void increase(double delta);
    void decrease(double delta);
};  

#endif




