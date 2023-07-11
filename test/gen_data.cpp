#include <pushplan/utils/data_gen.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/debug/visualizer_ros.h>
#include <ros/ros.h>

using namespace clutter;

void setupGlobals(const ros::NodeHandle& ph)
{
	ph.getParam("/fridge", FRIDGE);
	ph.getParam("robot/speed", R_SPEED);
	ph.getParam("occupancy_grid/res", DF_RES);
}

int main(int argc, char** argv)
{
	std::srand(std::time(0));

	ros::init(argc, argv, "whca");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	smpl::VisualizerROS visualizer(nh, 100);
	smpl::viz::set_visualizer(&visualizer);

	// Let publishers set up
	ros::Duration(1.0).sleep();

	// setup globals
	setupGlobals(ph);

	// setup DataGeneration
	DataGeneration dg;

	for (int i = 0; i < 2000; ++i)
	{
		dg.Reset();
		dg.GetPushData();
	}

	return 0;
}
