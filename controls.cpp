#include "MeOrion.h"
#include "controls.h"

BallControl::BallControl(PID *pid, Me7SegmentDisplay * disp) {
    myPID = pid;
    myDisp = disp;
}


ProportionalControl::ProportionalControl(PID *pid, Me7SegmentDisplay* disp): BallControl(pid, disp){}

void ProportionalControl::display(){
    Kp = myPID->GetKp();
    myDisp->display(Kp);
    myDisp->write(0, 0xF3);        
}
void ProportionalControl::increase(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Kp += delta;
    myPID->SetTunings(Kp, Ki, Kd);
}
void ProportionalControl::decrease(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Kp -= delta;
    myPID->SetTunings(Kp, Ki, Kd);
}

IntegralControl::IntegralControl(PID *pid, Me7SegmentDisplay* disp): BallControl(pid, disp){}

void IntegralControl::display(){
    Ki = myPID->GetKi();
    myDisp->display(Ki);
    myDisp->write(0, 0xB0);        
}
void IntegralControl::increase(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Ki += delta;
    myPID->SetTunings(Kp, Ki, Kd);
}
void IntegralControl::decrease(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Ki -= delta;
    myPID->SetTunings(Kp, Ki, Kd);
}

DerivitiveControl::DerivitiveControl(PID *pid, Me7SegmentDisplay* disp): BallControl(pid, disp){}

void DerivitiveControl::display(){
    Kd = myPID->GetKd();
    myDisp->display(Kd);
    myDisp->write(0, 0xDE);        
}
void DerivitiveControl::increase(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Kd += delta;
    myPID->SetTunings(Kp, Ki, Kd);
}
void DerivitiveControl::decrease(double delta){
    Kp = myPID->GetKp();
    Ki = myPID->GetKi();
    Kd = myPID->GetKd();
    Kd -= delta;
    myPID->SetTunings(Kp, Ki, Kd);
}

PIDControl::PIDControl(PID *pid, Me7SegmentDisplay* disp, double *position): BallControl(pid, disp){
    servoPosition = position;
}

void PIDControl::display(){
    int mode = myPID->GetMode();
    myDisp->display(mode);
    myDisp->write(0, 0xF3);        
    myDisp->write(1, 0xB0);        
    myDisp->write(2, 0xDE);        
}
void PIDControl::increase(double delta){
    int mode = myPID->GetMode();
    myPID->SetMode(!mode);
}
void PIDControl::decrease(double delta){
    int mode = myPID->GetMode();
    (*servoPosition) = 90;
    myPID->SetMode(!mode);
}

SetPointControl::SetPointControl(PID *pid, Me7SegmentDisplay* disp, double * p_setPoint): BallControl(pid, disp){
    setPoint = p_setPoint;
}

void SetPointControl::display(){
    int mode = myPID->GetMode();
    myDisp->display(*setPoint);
    myDisp->write(0, 0xEd); 
}
void SetPointControl::increase(double delta){
    if (*setPoint < 30){
        (*setPoint) += delta;
    }
}
void SetPointControl::decrease(double delta){
    if (*setPoint > 5){
        (*setPoint) -= delta;
    }
}

MeasurementControl::MeasurementControl(PID *pid, Me7SegmentDisplay* disp, double* measurement): BallControl(pid, disp){
    myMeasurement = measurement;
}
void MeasurementControl::display(){
    myDisp->display(*myMeasurement);
}
void MeasurementControl::increase(double delta){}
void MeasurementControl::decrease(double delta){}