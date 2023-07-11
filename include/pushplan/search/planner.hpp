#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <pushplan/agents/agent.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/search/mamo_search.hpp>
#include <pushplan/utils/bullet_sim.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/types.hpp>
#include <comms/ObjectsPoses.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>
#include <memory>

namespace clutter
{

class CBS;

namespace sampling
{
class SamplingPlanner;
} // namespace sampling

class Planner
{
public:
	Planner() : m_scene_id(-1),
				m_ph("~"), m_replan(true),
				m_plan_success(false), m_sim_success(false),
				m_push_input(false), m_rng(m_dev()), m_grasp_at(-1) {};
	~Planner();

	bool Init(const std::string& scene_file, int scene_id, bool ycb);
	bool Alive();
	bool SetupNGR();

	bool Plan(bool& done);
	bool FinalisePlan(
		const std::vector<ObjectState>& objects,
		std::vector<double>* start_state,
		trajectory_msgs::JointTrajectory& solution);
	bool PlanToHomeState(
		const std::vector<ObjectState>& objects,
		std::vector<double>* start_state,
		trajectory_msgs::JointTrajectory& solution);
	void AddGloballyInvalidPush(
		const std::pair<Coord, Coord>& bad_start_goal);
	void AddLocallyInvalidPush(
		unsigned int state_id, int agent_id, Coord bad_goal, int samples);

	bool TryExtract();
	std::uint32_t RunSolution();
	std::uint32_t RunSim(bool save=true);
	void AnimateSolution();

	bool SaveData();
	bool RunRRT();
	void RunStudy(int study);

	const std::shared_ptr<Agent>& GetAgent(const int& id);
	const std::vector<std::shared_ptr<Agent> >& GetAllAgents();

	bool Replan();
	const std::shared_ptr<CBS>& GetCBS() const;
	const std::shared_ptr<CollisionChecker>& GetCC() const;
	const std::shared_ptr<Robot>& GetRobot() const;
	const std::vector<std::pair<Coord, Coord> >* GetGloballyInvalidPushes() const;
	const std::map<Coord, int, coord_compare>* GetLocallyInvalidPushes(
		unsigned int state_id, int agent_id) const;
	const int& GetSceneID() const;
	const int& GetOoIID() const;
	const std::set<Coord, coord_compare>& GetNGR() const;
	const trajectory_msgs::JointTrajectory& GetFirstTraj() const;
	const bool& GetFirstTrajSuccess() const;

	// For KPIECE/RRT
	bool StateValidityChecker(const smpl::RobotState& state) {
		return m_robot->IsStateValid(state);
	};
	void GetStartState(smpl::RobotState& state) {
		m_robot->GetHomeState(state);
	}
	auto GoalPose() -> Eigen::Affine3d {
		return m_robot->PregraspPose();
	}
	bool ExecTraj(const trajectory_msgs::JointTrajectory& traj, int grasp_at=-1) {
		return m_sim->ExecTraj(traj, GetStartObjects(), grasp_at, m_ooi->GetID());
	}
	auto GetStartObjects() -> comms::ObjectsPoses
	{
		comms::ObjectsPoses start_objects;
		for (const auto& a: m_agents) {
			auto object = a->GetObject();

			comms::ObjectPose obj_pose;
			obj_pose.id = object->desc.id;
			obj_pose.xyz = { object->desc.o_x, object->desc.o_y, object->desc.o_z };
			obj_pose.rpy = { object->desc.o_roll, object->desc.o_pitch, object->desc.o_yaw };
			start_objects.poses.push_back(std::move(obj_pose));
		}

		return start_objects;
	}

	// For PP
	bool RunPP();
	fcl::CollisionObject* GetUpdatedObjectFromPriority(const LatticeState& s, int priority)
	{
		m_agents.at(m_priorities.at(priority))->UpdatePose(s);
		return m_agents.at(m_priorities.at(priority))->GetFCLObject();
	}

private:
	ros::NodeHandle m_ph, m_nh;

	std::string m_scene_file;
	std::shared_ptr<CollisionChecker> m_cc;
	std::shared_ptr<Robot> m_robot;
	std::shared_ptr<BulletSim> m_sim;
	std::shared_ptr<CBS> m_cbs;
	std::unique_ptr<MAMOSearch> m_mamo_search;
	std::shared_ptr<sampling::SamplingPlanner> m_sampling_planner;

	std::vector<std::shared_ptr<Agent> > m_agents;
	std::shared_ptr<Agent> m_ooi;
	std::unordered_map<int, size_t> m_agent_map;
	std::vector<double> m_goal;

	trajectory_msgs::JointTrajectory m_exec;
	std::vector<trajectory_msgs::JointTrajectory> m_rearrangements;
	comms::ObjectsPoses m_rearranged;

	HighLevelNode* m_cbs_soln;
	std::unordered_map<int, std::size_t> m_cbs_soln_map;
	std::map<std::string, double> m_stats, m_cbs_stats;

	std::vector<std::pair<Coord, Coord> > m_invalid_pushes_G;
	std::unordered_map<unsigned int, std::map<int, std::map<Coord, int, coord_compare> > > m_invalid_pushes_L;
	std::set<Coord, coord_compare> m_ngr;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::uniform_int_distribution<> m_distI;

	int m_scene_id, m_grasp_at, m_moved;
	bool m_replan, m_plan_success, m_sim_success, m_push_input, m_first_traj_success;
	double m_plan_budget, m_sim_budget, m_total_budget, m_timer;
	std::uint32_t m_violation;

	bool createCBS();
	bool createMAMOSearch();
	bool setupProblem();

	bool runSim();
	bool animateSolution();

	int cleanupLogs();
	void init_agents(
		bool ycb, std::vector<Object>& obstacles);
	void parse_scene(std::vector<Object>& obstacles);
	void read_solution();

	// For PP
	std::vector<size_t> m_priorities;
	void prioritise();
	void writeState(const std::string& prefix);
};

} // namespace clutter


#endif // PLANNER_HPP
