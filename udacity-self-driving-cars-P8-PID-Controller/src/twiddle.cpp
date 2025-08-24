#include "twiddle.h"
#include<math.h>
#include <iomanip>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double Kp_, double Kd_, double Ki_, double dKp_, double dKd_,
    double dKi_)
{
    PID::Init(Kp_, Kd_, Ki_);
    this->dKp = dKp_;
    this->dKd = dKd_;
    this->dKi = dKi_;
    this->best_err = std::numeric_limits<double>::max();
    this->curr_err = std::numeric_limits<double>::max();
    this->state = initialising;
    this->paramToBeUpdated = 0;
}

void Twiddle::setWS(uWS::WebSocket<uWS::SERVER> &ws_)
{
    this->ws = &ws_;
}

double Twiddle::getK(size_t idx) const
{
    double res = 0.;
    switch(idx)
    {
        case 0:
            res = this->Kp;
            //std::cout<<"idx is 0\n";
            break;
        case 1:
            res = this->Kd;
            break;
        case 2:
            res = this->Ki;
            break;
        default:
            res = 0.;
    }
    //std::cout<<this->Kp<<", "<<res<<std::endl;
    return res;

}

void Twiddle::setK(size_t idx, double val)
{
    switch(idx)
    {
        case 0:
            this->Kp = val;
            break;
        case 1:
            this->Kd = val;
            break;
        case 2:
            this->Ki= val;
            break;
        default:
            ;
    }
}

double Twiddle::getdK(size_t idx) const
{
    double res = 0.;
    switch(idx)
    {
        case 0:
            res = this->dKp;
            break;
        case 1:
            res = this->dKd;
            break;
        case 2:
            res = this->dKi;
            break;break;
        default:
            res = 0.;
    }
    return res;

}
void Twiddle::setdK(size_t idx, double val)
{
    switch(idx)
    {
        case 0:
            this->dKp = val;
            break;
        case 1:
            this->dKd = val;
            break;
        case 2:
            this->dKi= val;
            break;
        default:
            ;
    }
}

void Twiddle::checkTuningCondition()
{
    if (this->Kp+this->Kd+this->Ki<1e-10)
    {
        this->state = tuned;
        this->Kp = this->Kp;
        this->Kd = this->Kd;
        this->Ki = this->Ki;
    }
}


void Twiddle::reset()
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws->send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  this->curr_err = 0.;
  this->counter = 0;
}

double Twiddle::TotalError()
{
    static const size_t ITERATIONS = 4000;
    if (static_cast<size_t>(counter)>ITERATIONS && this->state != tuned)
    {
        switch (this->state)
        {
            case initialising:
                this->best_err = this->curr_err;
                this->state = increment_tuning;
                std::cout<<getK(paramToBeUpdated)<<std::endl;
                std::cout<<getdK(paramToBeUpdated)<<std::endl;
                setK(paramToBeUpdated, getK(paramToBeUpdated)+getdK(paramToBeUpdated));
                this->printK();
                break;
            case increment_tuning:
                if (this->curr_err<this->best_err)
                {
                    std::cout<<"iteration: "<<this->counter<<std::endl;
                    this->best_err = this->curr_err;
                    this->best_Kp = this->Kp;
                    this->best_Kd = this->Kd;
                    this->best_Ki = this->Ki;
                    setdK(paramToBeUpdated, 1.1*getdK(paramToBeUpdated));
                    this->printK();
                    paramToBeUpdated = (paramToBeUpdated+1)%3;
                }
                else
                {
                    setK(paramToBeUpdated, getK(paramToBeUpdated)-2.*getdK(paramToBeUpdated));
                    this->printK();
                    this->state = decrement_tuning;
                }
                reset();
                break;
            case decrement_tuning:
                if (this->curr_err<this->best_err)
                {
                    std::cout<<"iteration: "<<this->counter<<std::endl;
                    this->best_err = this->curr_err;
                    this->best_Kp = this->Kp;
                    this->best_Kd = this->Kd;
                    this->best_Ki = this->Ki;
                    setdK(paramToBeUpdated, 1.1*getdK(paramToBeUpdated));
                    this->printK();
                }
                else
                {
                    //set back to original value
                    setK(paramToBeUpdated, getK(paramToBeUpdated)+getdK(paramToBeUpdated));
                    setdK(paramToBeUpdated, 0.9*getdK(paramToBeUpdated));
                    this->printK();
                    this->state = decrement_tuning;
                }
                reset();
                paramToBeUpdated = (paramToBeUpdated+1)%3;
                break;
            case tuned:
                ;
            default:
                ;
        }
        this->checkTuningCondition();
        if (curr_err>best_err)
        {
            counter = ITERATIONS;
            std::cout<<"forcing next iteration\n";
            std::cout<<"curr_err: "<<this->curr_err<<"; best: "<< this->best_err<<std::endl;
        }
    }
    curr_err += pow(this->p_error, 2);
    double res = PID::TotalError();
    //force current iteration to stop

    return res;
}

void Twiddle::printK()
{
    std::cout<<std::setprecision (15)<<"best_err: "<<this->best_err<<std::endl;
    std::cout<<std::setprecision (15)<<"Kp: "<<this->Kp<<std::endl;
    std::cout<<std::setprecision (15)<<"Kd: "<<this->Kd<<std::endl;
    std::cout<<std::setprecision (15)<<"Ki: "<<this->Ki<<std::endl;
    std::cout<<std::setprecision (15)<<"dKp: "<<this->dKp<<std::endl;
    std::cout<<std::setprecision (15)<<"dKd: "<<this->dKd<<std::endl;
    std::cout<<std::setprecision (15)<<"dKi: "<<this->dKi<<std::endl;
}

bool Twiddle::isTuned()
{
    return this->state == tuned;
}

void Twiddle::setMaxError()
{
    curr_err = std::numeric_limits<double>::max();
}
