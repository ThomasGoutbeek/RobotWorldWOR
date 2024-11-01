#include "AbstractSensor.hpp"

#include "AbstractAgent.hpp"
#include "Robot.hpp"
#include "Shape2DUtils.hpp"

namespace Model
{
	/**
	 *
	 */
	AbstractSensor::AbstractSensor( AbstractAgent& anAgent) :
								agent{	&anAgent},
								running( false)
	{

	}
	/**
	 *
	 */
	void AbstractSensor::setOn( unsigned long aSleepTime /*= 100*/)
	{
		std::unique_lock< std::recursive_mutex > lock( sensorMutex);
		if (running == false)
		{
			running = true;
			std::thread newSensorThread( [this, aSleepTime]
										  {	run(aSleepTime);});
			sensorThread.swap( newSensorThread);
		}
	}
	/**
	 *
	 */
	void AbstractSensor::setOff()
	{
		std::unique_lock< std::recursive_mutex > lock( sensorMutex);

		running = false;
		//sensorThread.interrupt();
		sensorThread.join();
	}
	/**
	 *
	 */
	void AbstractSensor::sendPercept( std::shared_ptr< AbstractPercept > anAbstractPercept)
	{
		agent->addPercept( anAbstractPercept);
	}
	/**
	 *
	 */
	void AbstractSensor::run( unsigned long aSleepTime)
	{
		try
		{
			while (running == true)
			{
				Robot* robot = dynamic_cast<Robot*>(agent);
				if(robot)
				{
					robot->currentRadarPointCloud.clear();
					for(int i = 0;i<180;i++)
					{
						double angle = Utils::MathUtils::toRadians(2*i);
						std::shared_ptr< AbstractStimulus > currentStimulus = getStimulus(angle,robot->getPosition());
						std::shared_ptr< AbstractPercept > currentPercept = getPerceptFor( currentStimulus);
						sendPercept( currentPercept);
					}
				}
				std::this_thread::sleep_for( std::chrono::milliseconds( aSleepTime));
				
			}
		}
		catch (std::exception& e)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}

	
	/**
	 *
	 */
	void AbstractSensor::attachAgent( AbstractAgent& anAgent)
	{
		agent = &anAgent;
	}
	/**
	 *
	 */
	void AbstractSensor::detachAgent()
	{
		agent = nullptr;
	}
	/**
	 *
	 */
	std::string AbstractSensor::asString() const
	{
		return "AbstractSensor";
	}
	/**
	 *
	 */
	std::string AbstractSensor::asDebugString() const
	{
		return asString();
	}
} // namespace Model
