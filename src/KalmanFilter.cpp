#include "KalmanFilter.hpp"

const Matrix<double,2,2> naturalTransition{{1,0},{0,1}};
const Matrix<double,2,2> updateTransition{{1,0},{0,1}};


KalmanFilter::KalmanFilter(const Matrix<double,2,1>& states,const Matrix<double,2,2>& covariance):states(states),covariance(covariance)
{

}

const Matrix<double,2,1>& KalmanFilter::calcPredictedState(const Matrix<double,2,1>& states,const Matrix<double,2,1>& update)
{
    this->states = naturalTransition*states+updateTransition*update;
    return this->states;
}
const Matrix<double,2,2>& KalmanFilter::calcPredictedProcessCov(const Matrix<double,2,2>& covariance)
{
    this->covariance = naturalTransition*covariance*naturalTransition.transpose();
    return this->covariance;
}

const Matrix<double,2,2>& KalmanFilter::calcKalmanGain(const Matrix<double,2,2>& covariance,const Matrix<double,2,2>& sensorStd)
{
    this->kalmanGain = covariance*(sensorStd+covariance).inverse();
    return this->kalmanGain;
}

const Matrix<double,2,1>& KalmanFilter::calcAdjustedState(const Matrix<double,2,1>& states,const Matrix<double,2,2>& kalmanGain,const Matrix<double,2,1>& measurement)
{
    this->states = states+kalmanGain*(states-measurement);
    return this->states;
}

const Matrix<double,2,2>& KalmanFilter::calcAdjustedProcessCov(const Matrix<double,2,2>& covariance,const Matrix<double,2,2>& kalmanGain)
{
    this->covariance = (kalmanGain.identity()-kalmanGain)*covariance;
    return this->covariance;
}

wxPoint KalmanFilter::filter(const Matrix<double,2,1>& update,const Matrix<double,2,2>& sensorStd,const Matrix<double,2,1>& measurement)
{
    calcPredictedState(states,update);
    calcPredictedProcessCov(covariance);
    calcKalmanGain(covariance,sensorStd);
    calcAdjustedState(states,kalmanGain,measurement);
    calcAdjustedProcessCov(covariance,kalmanGain);

    wxPoint beliefPos(states[0][0],states[0][1]);
    return beliefPos;
}

void KalmanFilter::setStates(const Matrix<double,2,1>& states)
{
    this->states=states;
}