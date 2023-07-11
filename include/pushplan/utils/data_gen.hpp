#ifndef DATA_GEN_HPP
#define DATA_GEN_HPP

#include <pushplan/agents/robot.hpp>
#include <pushplan/utils/bullet_sim.hpp>

#include <ros/ros.h>

#include <memory>
#include <random>

namespace clutter
{

class DataGeneration
{
public:
	DataGeneration();
	void Reset();
	void GetPushData();

private:
	std::unique_ptr<Robot> m_robot;
	std::unique_ptr<BulletSim> m_sim;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	ros::NodeHandle m_ph, m_nh;
};

} // namespace clutter


#endif // DATA_GEN_HPP
