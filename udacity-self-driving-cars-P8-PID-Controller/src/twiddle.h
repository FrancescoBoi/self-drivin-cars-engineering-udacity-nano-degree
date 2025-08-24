#ifndef TWIDDLE_H
#define TWIDDLE_H
#include <uWS/uWS.h>
#include "PID.h"

enum twiddle_state {initialising, increment_tuning, decrement_tuning, tuned};

class Twiddle : public PID
{
public:
    /**
     * Constructor
     */
    Twiddle();

    /**
     * Destructor.
     */
    virtual ~Twiddle();
    void Init(double Kp_, double Kd_, double Ki_, double dKp_, double dKd_,
        double dKi);
    void setWS(uWS::WebSocket<uWS::SERVER> &ws_);
    double getK(size_t idx) const;
    void setK(size_t idx, double val);
    double getdK(size_t idx) const;
    void setdK(size_t idx, double val);
    void reset();
    void checkTuningCondition();
    void printK();
    double TotalError();
    bool isTuned();
    void setMaxError();
    twiddle_state state;
    size_t paramToBeUpdated;
    uWS::WebSocket<uWS::SERVER> *ws;


private:
    double dKp;
    double dKd;
    double dKi;
    double best_err;
    double curr_err;
    double best_Kp, best_Kd, best_Ki;

};
#endif
