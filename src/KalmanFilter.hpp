#ifndef KALMANFILTER_HPP_
#define KALMANFILTER_HPP_

#include "Matrix.hpp"
#include "Point.hpp"

class KalmanFilter
{
    public:
        KalmanFilter(const Matrix<double,2,1>& states,const Matrix<double,2,2>& covariance);
        virtual ~KalmanFilter()
        {

        };
        const Matrix<double,2,1>& calcPredictedState(const Matrix<double,2,1>& states,const Matrix<double,2,1>& update);
        const Matrix<double,2,2>& calcPredictedProcessCov(const Matrix<double,2,2>& covariance);
        const Matrix<double,2,2>& calcKalmanGain(const Matrix<double,2,2>& covariance,const Matrix<double,2,2>& sensorStd);
        const Matrix<double,2,1>& calcAdjustedState(const Matrix<double,2,1>& states,const Matrix<double,2,2>& kalmanGain,const Matrix<double,2,1>& measurement);
        const Matrix<double,2,2>& calcAdjustedProcessCov(const Matrix<double,2,2>& covariance,const Matrix<double,2,2>& kalmanGain);
        wxPoint filter(const Matrix<double,2,1>& update,const Matrix<double,2,2>& sensorStd,const Matrix<double,2,1>& measurement);
        void setStates(const Matrix<double,2,1>& states);
    private:
        Matrix<double,2,1> states;
        Matrix<double,2,2> covariance;
        Matrix<double,2,2> kalmanGain;
};

#endif