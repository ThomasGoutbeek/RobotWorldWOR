#ifndef PARTICLEFILTER_HPP_
#define PARTICLEFILTER_HPP_

#include "Matrix.hpp"
#include "Point.hpp"
#include "DistanceStimuli.hpp"
#include "DistancePercepts.hpp"
#include "LaserDistanceSensor.hpp"
#include <stdint.h>
#include <random>
#include <map>

using namespace Model;

class ParticleFilter{

    public:
        ParticleFilter();
        virtual ~ParticleFilter()
        {

        };
        void generateParticles();
        void generateRobotStimuli(wxPoint robotPos,const LaserDistanceSensor& sensor);
        void updateParticles(wxPoint measurement);
        void calcWeight(const LaserDistanceSensor& sensor);
        void getNewParticles();
        wxPoint getBeliefPos();
        void reverseWeights();
        const std::vector<wxPoint>& getParticles() const;
        void clearParticles();
    private:
        Matrix<double,2,1> states;
        std::vector<wxPoint> particles;
        std::vector<double> weights;
        std::vector<double> robotDistances;
        
};


#endif