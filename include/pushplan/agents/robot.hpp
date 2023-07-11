#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <pushplan/agents/object.hpp>
#include <pushplan/utils/types.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/bullet_sim.hpp>
#include <comms/ObjectsPoses.h>

#include <smpl/console/console.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/planning_params.h>
#include <smpl/debug/marker.h>
#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/occupancy_grid.h>
#include <smpl/debug/visualizer_ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/pushing_kdl_robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <string>
#include <memory>
#include <random>

namespace clutter
{

class HighLevelNode;
class Agent;

struct HashPush {
	size_t operator()(const std::tuple<ObjectState, Coord, int> &push_info) const;
};

struct EqualsPush
{
	bool operator()(
		const std::tuple<ObjectState, Coord, int> &a,
		const std::tuple<ObjectState, Coord, int> &b) const;
};

class Robot
{
public:
	Robot() : m_ph("~"), m_rng(m_dev()) {};
	void SetID(int id) { m_id = id; };
	int GetID() { return m_id; };

	bool Setup();
	void AddObstaclesFromSim();
	void SetMovables(const std::vector<std::shared_ptr<Agent> >& agents);
	void AddMovablesToCC();
	bool SetScene(const comms::ObjectsPoses& objects, const smpl::RobotState& state, bool vis=false);
	bool ProcessObstacles(const std::vector<Object>& obstacles, bool remove=false, bool movable=false);
	bool ProcessObstacles(const std::vector<Object*>& obstacles, bool remove=false, bool movable=false);
	void ProcessFCLObstacles(std::vector<Object> *obstacles, bool remove=false);
	void ProcessFCLObstacles(const std::vector<Object*> &obstacles, bool remove=false);

	bool SteerAction(
		const smpl::RobotState& to, int steps,
		const smpl::RobotState& from, const comms::ObjectsPoses& start_objs,
		smpl::RobotState& action_end, comms::ObjectsPoses& end_objs,
		double &frac, std::uint32_t& result, int planner=0);
	void GetRandomState(smpl::RobotState& s);

	bool Init();
	bool RandomiseStart();
	bool PlanApproachOnly(const std::vector<Object*>& movable_obstacles);
	bool PlanRetrieval(const std::vector<Object*>& movable_obstacles, bool finalise=false, smpl::RobotState* start_state=nullptr);
	bool PlanToHomeState(const std::vector<Object*>& movable_obstacles, smpl::RobotState* start_state=nullptr);
	void UpdateNGR(bool vis=false);
	bool SatisfyPath(HighLevelNode* ct_node, Trajectory** sol_path, int& expands, int& min_f);

	bool ComputeGrasps(
		const std::vector<double>& pregrasp_goal);
	void ConvertTraj(
		const Trajectory& traj_in,
		trajectory_msgs::JointTrajectory& traj_out);

	bool AtGrasp() {
		return m_t == m_grasp_at - 1;
	}
	bool AtEnd() {
		return m_t == m_solve.back().t;
	}
	int GraspAt() { return m_grasp_at; };

	bool UpdateKDLRobot(int mode);
	bool InitArmPlanner(bool interp=false);
	bool PlanPush(
		std::vector<double> *start_state,
		Agent* object, const std::vector<double>& push,
		const std::vector<Object*>& other_movables,
		const comms::ObjectsPoses& curr_scene,
		const double& push_frac,
		comms::ObjectsPoses& result,
		int& push_result,
		std::tuple<State, State, int>& debug_push,
		double &sim_time, bool input=false);
	bool GenMovablePush(
		Object& movable,
		std::vector<double>& push, double& move_dir, double& move_dist,
		Eigen::Affine3d& push_start_pose,
		int& push_result);
	void IdentifyReachableMovables(
		const std::vector<std::shared_ptr<Agent> >& agents,
		std::vector<int>& reachable_ids);

	const trajectory_msgs::JointTrajectory& GetLastPlanProfiled()
	{
		// m_planner->ProfilePath(m_rm.get(), m_traj);
		profileTrajectoryMoveIt(m_traj);
		return m_traj;
	};
	void ProfileTraj(trajectory_msgs::JointTrajectory& traj) {
		profileTrajectoryMoveIt(traj);
	}
	const trajectory_msgs::JointTrajectory& GetLastPlan() {
		return m_traj;
	};

	void SetSim(BulletSim* sim) {
		m_sim = sim;
	}
	void SetOOI(Object* ooi) {
		m_ooi = *ooi;
	}
	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	void CreateVirtualTable();

	const Trajectory* SolveTraj() const { return &m_solve; };

	State GetEEState(const State& state);
	const moveit_msgs::RobotState* GetStartState() { return &m_start_state; };
	smpl::RobotState* GetHomeState() { return &m_home_state; };

	void AnimateSolution();
	void PrintFK(const trajectory_msgs::JointTrajectory& traj)
	{
		for (const auto& p: traj.points)
		{
			auto pose = m_rm->computeFK(p.positions);
			SMPL_INFO("ee (x, y, z) = (%f, %f, %f)", pose.translation().x(), pose.translation().y(), pose.translation().z());
		}
	}

	auto GetStats() const -> const std::map<std::string, double>& {
		return m_stats;
	}

	auto ObsGrid() const -> const std::shared_ptr<smpl::OccupancyGrid>& {
		return m_grid_i;
	}
	auto NGRGrid() const -> const std::shared_ptr<smpl::OccupancyGrid>& {
		return m_grid_ngr;
	}

	void VizCC() {
		SV_SHOW_INFO(m_cc_i->getCollisionWorldVisualization());
	}

	auto TrajVoxels() const -> const std::vector<std::vector<Eigen::Vector3d>>* {
		return &m_traj_voxels;
	}

	smpl::PushingKDLRobotModel* RobotModel() {
		return m_rm.get();
	}

	void UpdateCCGroup(const std::string& group_name) {
		m_cc_i->UpdateGroup(group_name);
		m_cc_m->UpdateGroup(group_name);
	}
	void GetHomeState(smpl::RobotState& state) const {
		state = m_home_state;
	}
	void GetPregraspState(smpl::RobotState& state) const {
		state = m_pregrasp_state;
	}
	void GetGraspState(smpl::RobotState& state) const {
		state = m_grasp_state;
	}
	void GetPostgraspState(smpl::RobotState& state) const {
		state = m_postgrasp_state;
	}

	void ComputeFK(const smpl::RobotState& q, Eigen::Affine3d& ee_pose) {
		ee_pose = m_rm->computeFK(q);
	}
	void ComputeJacobian(const smpl::RobotState& q, Eigen::MatrixXd& Jq) {
		m_rm->computeJacobian(q, Jq);
	}

	void RunManipulabilityStudy(int N=1000);
	void RunPushIKStudy(int N=25);
	void VisPlane(double z);
	bool SavePushDebugData(int scene_id);

	// For KPIECE
	bool ComputeGraspTraj(const smpl::RobotState& state, trajectory_msgs::JointTrajectory& grasp_traj);
	bool CheckGraspTraj(const smpl::RobotState& state, const comms::ObjectsPoses& objects);
	auto GetPlanningJoints() const -> const std::vector<std::string>& {
		return m_robot_config.planning_joints;
	}
	auto GetRCM() const -> smpl::collision::RobotCollisionModelConstPtr {
		return m_cc_i->robotCollisionModel();
	}
	auto GetRMCM() const -> smpl::collision::RobotMotionCollisionModelConstPtr {
		return m_cc_i->robotMotionCollisionModel();
	}
	bool IsStateValid(const smpl::RobotState& state) {
		return m_cc_i->isStateValid(state);
	}
	auto PregraspPose() -> Eigen::Affine3d {
		return m_rm->computeFK(m_pregrasp_state);
	}

private:
	int m_id;
	ros::NodeHandle m_nh, m_ph;
	std::string m_robot_description, m_planning_frame;
	RobotModelConfig m_robot_config;
	std::unique_ptr<smpl::PushingKDLRobotModel> m_rm;
	moveit_msgs::RobotState m_start_state;
	ros::Publisher m_vis_pub;

	// cached from robot model
	std::vector<double> m_min_limits;
	std::vector<double> m_max_limits;
	std::vector<bool> m_continuous;
	std::vector<bool> m_bounded;

	std::vector<int> m_coord_vals;
	std::vector<double> m_coord_deltas;

	int m_grasp_at;
	double m_table_z;
	Eigen::Affine3d m_home_pose, m_pregrasp_pose, m_grasp_pose, m_postgrasp_pose;
	smpl::RobotState m_home_state, m_pregrasp_state, m_grasp_state, m_postgrasp_state;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;
	std::normal_distribution<> m_distG;

	std::shared_ptr<smpl::DistanceMapInterface> m_df_i, m_df_m, m_df_ngr;
	std::shared_ptr<smpl::OccupancyGrid> m_grid_i, m_grid_m, m_grid_ngr;
	std::unique_ptr<smpl::collision::CollisionSpace> m_cc_i, m_cc_m;
	double m_ngr_res;
	Eigen::Vector3d m_ngr_origin, m_ngr_gmin, m_ngr_gmax;

	PlannerConfig m_planning_config;
	smpl::PlanningParams m_planning_params;
	std::unique_ptr<smpl::PlannerInterface> m_planner;
	bool m_planner_init;
	std::vector<double> m_goal_vec;
	moveit_msgs::Constraints m_goal;
	std::string m_chain_tip_link;
	trajectory_msgs::JointTrajectory m_traj;
	std::vector<std::vector<Eigen::Vector3d>> m_traj_voxels;

	int m_t;
	Trajectory m_solve;
	Object m_ooi;
	std::vector<std::shared_ptr<Agent> > m_movables; // pointers to relevant objects
	std::unordered_map<int, size_t> m_movable_map;
	std::vector<moveit_msgs::CollisionObject> m_movable_moveit_objs;
	std::shared_ptr<CollisionChecker> m_cc;

	BulletSim* m_sim = nullptr;
	std::vector<trajectory_msgs::JointTrajectory> m_push_trajs, m_push_actions;
	int m_grasp_tries, m_invvel_iters;
	double m_plan_push_time, m_grasp_lift, m_grasp_z, m_Kp, m_Ki, m_Kd, m_dt;

	std::map<std::string, double> m_stats, m_debug_push_info;
	double m_planner_time, m_sim_time;

	std::unordered_map<
		std::tuple<ObjectState, Coord, int>,
		std::vector<std::tuple<comms::ObjectsPoses, comms::ObjectsPoses, std::vector<int>, trajectory_msgs::JointTrajectory> >,
		HashPush,
		EqualsPush> m_valid_sims;

	void getPushStartPose(
		const std::vector<double>& push,
		Eigen::Affine3d& push_pose,
		bool input);
	void samplePushStartPose(
		std::vector<double>& push, Object& movable,
		Eigen::Affine3d& start_pose);
	bool planToPoseGoal(
		const moveit_msgs::RobotState& start_state,
		const Eigen::Affine3d& pose_goal,
		trajectory_msgs::JointTrajectory& push_traj,
		double t=0.1);
	int computePushAction(
		const double time_start,
		const smpl::RobotState& jnt_positions,
		const smpl::RobotState& jnt_velocities,
		const Eigen::Affine3d& end_pose,
		trajectory_msgs::JointTrajectory& action);
	int computePushPath(
		const double time_start,
		const smpl::RobotState& jnt_positions,
		const smpl::RobotState& jnt_velocities,
		const Trajectory* path,
		trajectory_msgs::JointTrajectory& action);

	bool reinitStartState();

	void coordToState(const Coord& coord, State& state) const;
	void stateToCoord(const State& state, Coord& coord) const;

	// Robot Model functions
	bool readRobotModelConfig(const ros::NodeHandle &nh);
	bool setupRobotModel();
	bool readStartState();
	bool setReferenceStartState();
	bool setGripper(bool open);
	bool readResolutions(std::vector<double>& resolutions);

	void initObjects();
	void reinitObjects(const State& s);

	double profileAction(
		const smpl::RobotState& parent,
		const smpl::RobotState& succ);

	void initOccupancyGrids();
	bool initCollisionChecker();
	bool getCollisionObjectMsg(
		const Object& object,
		moveit_msgs::CollisionObject& obj_msg,
		bool remove);
	bool processCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable);
	bool processSTLMesh(
		const Object& object, bool remove, bool movable);

	bool addCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable);
	bool removeCollisionObjectMsg(
		const moveit_msgs::CollisionObject& object, bool movable);
	bool checkCollisionObjectSanity(
		const moveit_msgs::CollisionObject& object) const;
	auto findCollisionObject(const std::string& id, bool movable) const
		-> smpl::collision::CollisionObject*;
	bool setCollisionRobotState();

	auto makePathVisualisation() const
	-> std::vector<smpl::visual::Marker>;

	bool initPlanner();
	bool readPlannerConfig(const ros::NodeHandle &nh);
	bool createPlanner(bool interp);
	void fillGoalConstraint();
	void createMultiPoseGoalConstraint(moveit_msgs::MotionPlanRequest& req);
	void createPoseGoalConstraint(
		const Eigen::Affine3d& pose,
		moveit_msgs::MotionPlanRequest& req);
	void createJointSpaceGoal(
		const smpl::RobotState& pose,
		moveit_msgs::MotionPlanRequest& req);
	void addPathConstraint(moveit_msgs::Constraints& path_constraints);
	bool getStateNearPose(
		const Eigen::Affine3d& pose,
		const smpl::RobotState& seed_state,
		smpl::RobotState& state,
		int N=2,
		const std::string& ns="");

	bool attachAndCheckObject(
		const Object& object, const smpl::RobotState& state);
	bool attachObject(const Object& obj);
	bool detachObject();
	void displayObjectMarker(const Object& object);

	bool planApproach(
		const std::vector<std::vector<double> >& approach_cvecs,
		moveit_msgs::MotionPlanResponse& res,
		const std::vector<Object*>& movable_obstacles,
		bool finalise=false,
		smpl::RobotState* start_state=nullptr);
	bool planRetract(
		const std::vector<std::vector<double> >& retract_cvecs,
		moveit_msgs::MotionPlanResponse& res,
		const std::vector<Object*>& movable_obstacles,
		bool finalise=false);

	void getTrajSpheres(
		const trajectory_msgs::JointTrajectory& traj,
		std::set<std::vector<double> >& spheres);
	void voxeliseTrajectory();

	int getPushIdx(double push_frac);
	void addPushToDB(
		Object* o, const Coord &goal, const int& aidx,
		const comms::ObjectsPoses &init_scene,
		const comms::ObjectsPoses &result_scene,
		const std::vector<int> &relevant_ids,
		const trajectory_msgs::JointTrajectory& push_action);
	bool lookupPushInDB(
		Object* o, const Coord &goal, const int& aidx, const comms::ObjectsPoses &curr_scene,
		comms::ObjectsPoses &new_scene,
		trajectory_msgs::JointTrajectory &new_action);

	// for moveit profiling
	moveit::core::RobotModelPtr m_moveit_robot_model;
	moveit::core::RobotStatePtr m_moveit_robot_state;
	robot_trajectory::RobotTrajectoryPtr m_moveit_trajectory_ptr;
	void profileTrajectoryMoveIt(trajectory_msgs::JointTrajectory& traj);

	void visMAPFPath(const Trajectory* path);
};

} // namespace clutter


#endif // ROBOT_HPP
