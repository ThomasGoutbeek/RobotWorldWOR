#include "ParticleFilter.hpp"

#define MAX_SCREENSIZE 1024
#define NR_OF_PARTICLES 1000
#define NR_OF_LASERS 180

ParticleFilter::ParticleFilter()
{

}

void ParticleFilter::generateParticles()
{
    std::random_device rd{};
	std::mt19937 gen{rd()};
	std::uniform_int_distribution<int> dist{0,MAX_SCREENSIZE};
	for(int i = 0;i<NR_OF_PARTICLES;i++)
    {
        particles.push_back(wxPoint(dist(gen),dist(gen)));
    }
}

void ParticleFilter::generateRobotStimuli(wxPoint robotPos,const LaserDistanceSensor& sensor)
{
    for(int i = 0;i<NR_OF_LASERS;i++)
    {
        double angle = Utils::MathUtils::toRadians(2*i);
        std::shared_ptr<AbstractStimulus> stimulus = sensor.getStimulus(angle,robotPos);
        DistanceStimulus* distStimulus = dynamic_cast< DistanceStimulus* >( stimulus.get());
        robotDistances.push_back(distStimulus->distance);
    }
}
void ParticleFilter::updateParticles(wxPoint measurement){
    for(std::size_t i = 0;i<NR_OF_PARTICLES;i++)
    {
        particles.at(i).x = particles.at(i).x + measurement.x;
        particles.at(i).y = particles.at(i).y + measurement.y;
    }
}

void ParticleFilter::calcWeight(const LaserDistanceSensor& sensor)
{
    std::vector<double> angles(NR_OF_LASERS);
    for (std::size_t j = 0; j < NR_OF_LASERS; ++j) {
        angles[j] = Utils::MathUtils::toRadians(2 * j);
    }
    
    for(std::size_t i = 0;i<NR_OF_PARTICLES;i++)
    {
        double weight = 0;
        for(std::size_t j=0;j<NR_OF_LASERS;j++)
        {
            std::shared_ptr<AbstractStimulus> stimulus = sensor.getStimulus(angles.at(j),particles.at(i));
            DistanceStimulus* distStimulus = dynamic_cast< DistanceStimulus* >( stimulus.get());
            double particleDistance = distStimulus->distance;
            weight =  weight + std::abs(robotDistances.at(j)-particleDistance);
        }
        weights.push_back(weight);
    }
    robotDistances.clear();
}
void ParticleFilter::getNewParticles()
{
    for (std::size_t i = 1; i < weights.size(); i++) {
        weights[i] += weights[i - 1];
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> d(0.0,1.0);
    std::map<int, int> map;
    std::vector<wxPoint> oldParticles = particles;

    particles.clear();
    std::uniform_int_distribution<int> offset_dist(-5, 5);
 // Random offset range
    for (int i =0;i<NR_OF_PARTICLES;i++) {
        double random_number = d(gen);
        auto it = std::lower_bound(weights.begin(), weights.end(), random_number);
        int index = std::distance(weights.begin(), it);
        wxPoint new_particle = oldParticles.at(index);
        new_particle.x += offset_dist(gen);
        new_particle.y += offset_dist(gen);
        particles.push_back(new_particle);
    }
    weights.clear();

}
wxPoint ParticleFilter::getBeliefPos()
{
    double sum_x = 0;
    double sum_y = 0;
    for(wxPoint particle:particles)
    {
        sum_x = sum_x + particle.x;
        sum_y = sum_y + particle.y;
    }
    return wxPoint(sum_x/NR_OF_PARTICLES,sum_y/NR_OF_PARTICLES);
}
void ParticleFilter::reverseWeights()
{
    double oldMin = *min_element(weights.begin(),weights.end());
    double oldMax = *max_element(weights.begin(),weights.end());
    double newMin = oldMax;
    double newMax = oldMin;
    double oldRange = (oldMax - oldMin);
    double newRange = (newMax - newMin);

    double totalWeights = 0;

    for(double& weight:weights)
    {
        weight = (((weight - oldMin) * newRange) / oldRange) + newMin;
        totalWeights = totalWeights + weight;
    }
    for(double& weight:weights)
    {
        weight = weight/totalWeights;
    }
}

const std::vector<wxPoint>& ParticleFilter::getParticles() const
{
    return particles;
}

void ParticleFilter::clearParticles()
{
    particles.clear();
}