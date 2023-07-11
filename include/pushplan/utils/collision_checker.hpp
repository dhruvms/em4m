#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <pushplan/agents/object.hpp>
#include <pushplan/utils/types.hpp>

#include <fcl/broadphase/broadphase.h>

#include <vector>
#include <random>
#include <unordered_map>
#include <utility>
#include <iostream>

namespace std
{

class PairHash
{
public:
	// id is returned as hash function
	size_t operator()(const pair<int, int>& s) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, s.first);
		boost::hash_combine(seed, s.second);
		return seed;
	}
};

inline
bool operator==(const pair<int, int>& a, const pair<int, int>& b)
{
	return (a.first == b.first && a.second == b.second) ||
				(a.first == b.second && a.second == b.first);
}

inline
ostream& operator<<(ostream& os, unordered_map<pair<int, int>, int, PairHash> const& s)
{
	os << "[" << s.size() << "] { ";
	for (pair<pair<int, int>, int> i : s)
		os << "(" << i.first.first << ", " << i.first.second << ", " << i.second << ") ";
	return os << "}\n";
}

} // namespace std

namespace clutter
{

class Planner;
class Robot;
class Agent;

class CollisionChecker
{
public:
	CollisionChecker(Planner* planner, const std::vector<Object>& obstacles, std::shared_ptr<Robot> r);
	void ReinitMovableCC(const std::vector<std::shared_ptr<Agent> >& movables);

	void AddObstacle(const Object& o)
	{
		m_obstacles.push_back(o);

		LatticeState s;
		s.state.push_back(m_obstacles.back().desc.o_x);
		s.state.push_back(m_obstacles.back().desc.o_y);
		m_obstacles.back().UpdatePose(s);
		m_fcl_immov->registerObject(m_obstacles.back().GetFCLObject());

		m_fcl_immov->setup();
	};
	void AddObstacle(Object* o) {
		AddObstacle(*o);
	}
	void SetRobot(std::shared_ptr<Robot> r) { m_robot = r; };

	void UpdateTraj(const int& priority, const Trajectory& traj);

	bool OutOfBounds(const LatticeState& s);
	bool ImmovableCollision(const State& s, fcl::CollisionObject* o);
	bool ImmovableCollision(fcl::CollisionObject* o);
	bool ObjectObjectCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2);
	bool ObjectObjectCollision(fcl::CollisionObject* o1, const int& a2_id, const LatticeState& a2_q);
	bool ObjectObjectsCollision(
			fcl::CollisionObject* o1,
			const std::vector<int>& other_ids,
			const std::vector<LatticeState>& other_poses);
	double ObstacleGaussianCost(double x, double y) const;
	double ObstacleDist(fcl::CollisionObject* o);
	bool RobotObjectCollision(
		Agent* a1, const LatticeState& a1_state,
		const LatticeState& robot_state,
		int t, bool process=true);

	State GetRandomStateOutside(fcl::CollisionObject* o);

	double GetTableHeight() { return m_obstacles.at(m_base_loc).desc.o_z + m_obstacles.at(0).desc.z_size; };
	double OutsideXMin() { return m_obstacles.at(m_base_loc).desc.o_x - (2 * m_obstacles.at(m_base_loc).desc.x_size); };
	double OutsideYMin() { return m_obstacles.at(m_base_loc).desc.o_y - (0.67 * m_obstacles.at(m_base_loc).desc.y_size); };
	double OutsideXMax() { return m_obstacles.at(m_base_loc).desc.o_x - m_obstacles.at(m_base_loc).desc.x_size; };
	double OutsideYMax() { return m_obstacles.at(m_base_loc).desc.o_y + (0.67 * m_obstacles.at(m_base_loc).desc.y_size); };

	int NumObstacles() { return (int)m_obstacles.size(); };
	std::vector<Object>* GetObstacles() { return &m_obstacles; };

	// PP
	bool PPCollisionCheck(
		Agent* a, const LatticeState& s, fcl::CollisionObject* o,
		const int& priority, bool goal_check);

	void ReleaseRobot() { m_robot.reset(); };

private:
	Planner* m_planner = nullptr;

	std::vector<Object> m_obstacles;
	std::shared_ptr<Robot> m_robot;
	size_t m_base_loc;
	std::vector<Trajectory> m_trajs;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	fcl::BroadPhaseCollisionManager* m_fcl_immov = nullptr;
	fcl::BroadPhaseCollisionManager* m_fcl_mov = nullptr;

	void cleanupChildren(std::vector<int>& check);

	bool checkCollisionObjSet(
		const Object& o1, const State& o1_loc,
		bool rect_o1, const std::vector<State>& o1_rect,
		const std::vector<Object>* a2_objs);

	bool rectRectCollision(
		const std::vector<State>& r1, const std::vector<State>& r2);
	bool rectCircCollision(
		const std::vector<State>& r1,
		const Object& c1, const State& c1_loc);
	bool circCircCollision(
		const Object& c1, const State& c1_loc,
		const Object& c2, const State& c2_loc);

};

} // namespace clutter


#endif // COLLISION_CHECKER_HPP
