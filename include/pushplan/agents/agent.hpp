#ifndef AGENT_HPP
#define AGENT_HPP

#include <pushplan/agents/object.hpp>
#include <pushplan/agents/agent_lattice.hpp>
#include <pushplan/search/cbs_nodes.hpp>
#include <pushplan/search/conflicts.hpp>
#include <pushplan/utils/types.hpp>
#include <pushplan/utils/collision_checker.hpp>

#include <smpl/distance_map/distance_map_interface.h>
#include <smpl/occupancy_grid.h>
#include <ros/ros.h>

#include <vector>
#include <string>
#include <memory>

namespace clutter
{

struct Eigen_Vector3d_compare
{
	bool operator()(const Eigen::Vector3d& u, const Eigen::Vector3d& v) const
	{
		return std::tie(u.x(), u.y(), u.z()) < std::tie(v.x(), v.y(), v.z());
	}
};

class Agent
{
public:
	Agent() : m_ph("~"), m_set(false), m_pp(false) {};
	Agent(const Object& o) : m_ph("~"), m_set(true), m_pp(false)
	{
		m_obj = o;
		m_obj_desc = o.desc;
	};
	void SetObject(const Object& o)
	{
		m_obj = o;
		m_obj_desc = o.desc;
		m_set = true;
	}

	bool ResetObject();
	bool SetObjectPose(
		const std::vector<double>& xyz,
		const std::vector<double>& rpy);
	bool SetObjectPose(const ContPose& pose);

	bool Init();
	void ComputeNGRComplement(
		double ox, double oy, double oz,
		double sx, double sy, double sz, bool vis=false);

	void ResetInvalidPushes(
		const std::vector<std::pair<Coord, Coord> >* invalids_G,
		const std::map<Coord, int, coord_compare>* invalids_L);
	// const bgi::rtree<value, bgi::quadratic<8> >& GetInvalidPushes() const;
	void AddHallucinatedConstraint(const Coord &c);
	int InvalidPushCount(const Coord &c);

	bool SatisfyPath(
		HighLevelNode* ct_node,
		Trajectory** sol_path,
		int& expands,
		int& min_f,
		std::unordered_set<int>* to_avoid = nullptr);
	Trajectory* SolveTraj() { return &m_solve; };
	const Trajectory* SolveTraj() const { return &m_solve; };
	void SetSolveTraj(const Trajectory& solve) { m_solve = solve; };

	void SetCC(const std::shared_ptr<CollisionChecker>& cc) {
		m_cc = cc;
	}
	void SetObstacleGrid(const std::shared_ptr<smpl::OccupancyGrid>& obs_grid) {
		m_obs_grid = obs_grid;
	}
	void SetNGRGrid(const std::shared_ptr<smpl::OccupancyGrid>& ngr_grid) {
		m_ngr_grid = ngr_grid;
	}
	void ResetSolution() {
		m_solve.clear();
	}

	bool GetSE2Push(std::vector<double>& push, bool input=false);
	void GetVoxels(const ContPose& pose, std::set<Coord, coord_compare>& voxels);
	const int& GetID() const { return m_obj.desc.id; };

	Object* GetObject() { return &m_obj; };
	fcl::CollisionObject* GetFCLObject() { return m_obj.GetFCLObject(); };
	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		m_obj.GetMoveitObj(msg);
	};

	void UpdatePose(const LatticeState& s);
	bool OutOfBounds(const LatticeState& s);
	bool ImmovableCollision();
	bool ObjectObjectCollision(const int& a2_id, const LatticeState& a2_q);
	bool ObjectObjectsCollision(
			const std::vector<int>& other_ids,
			const std::vector<LatticeState>& other_poses);
	double ObstacleGaussianCost(double x, double y) const;
	bool OutsideNGR(const LatticeState& s);
	double ObsDist(double x, double y);

	void VisualiseState(const Coord& s, const std::string& ns="", int hue=180);
	auto VisualiseState(const LatticeState& s, const std::string& ns="", int hue=180, bool vis=true)
		-> std::vector<smpl::visual::Marker>;

	Coord Goal() const { return m_goal; };
	auto InitState() const -> const LatticeState& { return m_init; };

	bool Set() { return m_set; };

	// PP
	bool InitPP();
	bool PlanPrioritised(int p);
	bool PrioritisedCollisionCheck(const LatticeState& s, bool goal_check=false);
	bool PP() { return m_pp; };

private:
	ros::NodeHandle m_ph;
	Object m_obj;
	ObjectDesc m_obj_desc;
	LatticeState m_init;
	Coord m_goal;
	Trajectory m_solve;
	bool m_set;
	std::vector<double> m_input_push;
	std::unique_ptr<AgentLattice> m_lattice;
	std::string m_planning_frame;

	std::shared_ptr<smpl::OccupancyGrid> m_obs_grid, m_ngr_grid;
	std::set<Eigen::Vector3d, Eigen_Vector3d_compare> m_ngr_complement;
	std::vector<LatticeState> m_ngr_complement_states;

	State m_goalf;

	std::shared_ptr<CollisionChecker> m_cc;
	std::unique_ptr<Search> m_search;

	// PP
	bool m_pp;
	int m_priority, m_t;

	bool computeGoal();
	bool createLatticeAndSearch();

	// check collisions with static obstacles
	bool stateObsCollision(const LatticeState& s);

	bool updateObjectTransform(const LatticeState& s);
	// check collisions with NGR
	bool stateOutsideNGR(const LatticeState& s, double &dist);
	bool stateOutsideNGR(const LatticeState& s);

	auto makePathVisualization()
		-> std::vector<smpl::visual::Marker>;
};

} // namespace clutter


#endif // AGENT_HPP
