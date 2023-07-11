#include <pushplan/search/planner.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/discretisation.hpp>

#include <smpl/debug/visualizer_ros.h>
#include <ros/ros.h>

#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>

using namespace clutter;

void readDiscretisationParams(const ros::NodeHandle& ph, WorldResolutionParams& disc_params)
{
	std::string disc_string;
	if (!ph.getParam("objects/discretisation", disc_string)) {
		throw std::runtime_error("Parameter 'objects/discretisation' not found in planning params");
	}

	auto disc = ParseMapFromString<double>(disc_string);
	SetWorldResolutionParams(
		disc["x"], disc["y"], disc["theta"],
		disc["ox"], disc["oy"], disc_params);
	DiscretisationManager::Initialize(disc_params);
}

void setupGlobals(const ros::NodeHandle& ph)
{
	ph.getParam("/fridge", FRIDGE);
	ph.getParam("mapf/planning_time", MAPF_PLANNING_TIME);
	ph.getParam("mapf/res", RES);
	ph.getParam("mapf/goal_thresh", GOAL_THRESH);
	ph.getParam("mapf/whca/window", WINDOW);
	ph.getParam("mapf/whca/grid", GRID);
	ph.getParam("robot/speed", R_SPEED);
	ph.getParam("goal/save", SAVE);
	ph.getParam("goal/cc_2d", CC_2D);
	ph.getParam("goal/cc_3d", CC_3D);
	ph.getParam("occupancy_grid/res", DF_RES);
	ph.getParam("robot/pushing/samples", SAMPLES);

	int llhc, hlhc, cp, algo;
	ph.getParam("mapf/cbs/llhc", llhc);
	ph.getParam("mapf/cbs/hlhc", hlhc);
	ph.getParam("mapf/cbs/cp", cp);
	ph.getParam("mapf/cbs/algo", algo);
	LLHC = static_cast<LowLevelConflictHeuristic>(llhc);
	HLHC = static_cast<HighLevelConflictHeuristic>(hlhc);
	CP = static_cast<ConflictPrioritisation>(cp);
	ALGO = static_cast<MAPFAlgo>(algo);
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
	WorldResolutionParams disc_params;
	setupGlobals(ph);
	readDiscretisationParams(ph, disc_params);

	// read from NONE file
	std::string filename(__FILE__), results(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../dat/FIRST.txt";
	results = results.substr(0, found + 1) + "../dat/RESULTS.csv";

// int runs;
// ph.getParam("robot/runs", runs);
// for (int i = 0; i < runs; ++i)
// {
	std::ifstream NONE;
	NONE.open(filename);

	if (NONE.is_open())
	{
		std::string line, level;
		while (!NONE.eof())
		{
			getline(NONE, line);
			if (line.length() == 0) {
				break;
			}
			int scene_id = std::stoi(line);
			if (scene_id < 100000)
			{
				level = "0";
				// ROS_WARN("Planning for a scene with no movable objects!");
			}
			else if (scene_id < 200000) {
				level = "5";
			}
			else if (scene_id < 300000) {
				level = "10";
			}
			else if (scene_id < 400000) {
				level = "15";
			}
			else {
				level = "nb";
			}

			std::string planfile(__FILE__);
			auto found = planfile.find_last_of("/\\");
			planfile = planfile.substr(0, found + 1) + "../../../../simplan/src/simplan/data/clutter_scenes/";
			planfile += level + "/plan_" + line + "_SCENE.txt";

			bool replay;
			ph.getParam("robot/replay", replay);

			int runs;
			ph.getParam("robot/runs", runs);
			ROS_WARN("Run planner %d times on: %s", runs, planfile.c_str());
for (int i = 0; i < runs; ++i)
{
			Planner p;
			bool ycb;
			ph.getParam("objects/ycb", ycb);
			if (ycb) {
				scene_id = -1;
			}
			if (!p.Init(planfile, scene_id, ycb)) {
				continue;
			}
			ROS_INFO("Planner and simulator init-ed!");

			if (!replay)
			{
				bool done;
				if (p.Plan(done))
				{
					// p.RunSim(SAVE);
					// if (SAVE) {
					// 	p.SaveData();
					// }
				}
			}

			else {
				p.RunSolution();
			}
}
		}
	}
	else
	{
		ROS_ERROR("Planner init error");
		return false;
	}

	NONE.close();
// }

	return 0;
}
