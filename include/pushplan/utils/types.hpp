#ifndef TYPES_HPP
#define TYPES_HPP

#include <pushplan/utils/constants.hpp>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <boost/functional/hash.hpp>

#include <ostream>
#include <vector>
#include <memory>

namespace clutter
{

typedef std::vector<int> Coord;
typedef std::vector<double> State;

struct coord_compare
{
	bool operator()(const Coord& u, const Coord& v) const
	{
		if (u.size() != v.size())
		{
			assert(false);
			return u.size() < v.size();
		}
		else if (u.size() == 2) {
			return std::tie(u.at(0), u.at(1)) < std::tie(v.at(0), v.at(1));
		}
		else if (u.size() == 3) {
			return std::tie(u.at(0), u.at(1), u.at(2)) < std::tie(v.at(0), v.at(1), v.at(2));
		}
		else {
			assert(false);
			return false;
		}
	}
};

struct coord_hash
{
	size_t operator()(const Coord& c) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, boost::hash_range(c.begin(), c.begin() + 2));
		return seed;
	}
};

// bool operator==(
// 	const std::map<Coord, int, coord_compare> &lhs,
// 	const std::map<Coord, int, coord_compare> &rhs);

struct LatticeState
{
	Coord coord;
	State state;
	int t, hc;
};

typedef std::vector<LatticeState> Trajectory;
typedef std::vector<LatticeState*> STATES;

bool operator==(const LatticeState& a, const LatticeState& b);
bool operator!=(const LatticeState& a, const LatticeState& b);
bool operator==(
	const std::vector<std::pair<int, Trajectory> >& a,
	const std::vector<std::pair<int, Trajectory> >& b);
bool operator!=(
	const std::vector<std::pair<int, Trajectory> >& a,
	const std::vector<std::pair<int, Trajectory> >& b);

} // namespace clutter

namespace std {

template <>
struct hash<clutter::LatticeState>
{
	typedef clutter::LatticeState argument_type;
	typedef std::size_t result_type;
	result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace clutter
{

class Search
{
public:
	virtual int set_start(int start_id) = 0;
	virtual int set_goal(int goal_id) = 0;
	virtual std::size_t push_start(int start_id) = 0;
	virtual std::size_t push_goal(int goal_id) = 0;

	virtual void set_max_planning_time(double max_planning_time_ms) = 0;
	virtual int get_n_expands() const = 0;
	virtual unsigned int get_min_f() const = 0;
	virtual void reset() = 0;

	virtual int replan(
		std::vector<int>* solution_path, int* solution_cost) = 0;

protected:
	std::vector<int> m_start_ids;
	std::vector<int> m_goal_ids;

	// Search statistics
	double m_search_time;
	int *m_expands; // expansions per queue
	int m_solution_cost;
	unsigned int m_min_f;
};

struct RobotModelConfig
{
	std::string group_name;
	std::vector<std::string> planning_joints;
	std::string kinematics_frame;
	std::string chain_tip_link;
	std::vector<std::string> push_links;
	std::vector<std::string> gripper_joints;
};

struct PlannerConfig
{
	std::string discretization;
	std::string mprim_filename;
	bool use_xyz_snap_mprim;
	bool use_rpy_snap_mprim;
	bool use_xyzrpy_snap_mprim;
	bool use_short_dist_mprims;
	double xyz_snap_dist_thresh;
	double rpy_snap_dist_thresh;
	double xyzrpy_snap_dist_thresh;
	double short_dist_mprims_thresh;
};

} // namespace clutter

#endif // TYPES_HPP
