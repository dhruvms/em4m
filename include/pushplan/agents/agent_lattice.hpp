#ifndef AGENT_LATTICE_HPP
#define AGENT_LATTICE_HPP

#include <pushplan/search/cbs_nodes.hpp>
#include <pushplan/utils/types.hpp>

#include <smpl/types.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <vector>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace clutter
{

class Agent;

class AgentLattice
{
public:

	void init(Agent* agent);
	void reset();

	int PushStart(const LatticeState& s);
	int PushGoal(const Coord& p);

	void SetCTNode(HighLevelNode* ct_node);
	void AvoidAgents(const std::unordered_set<int>& to_avoid);
	void ResetInvalidPushes(
		const std::vector<std::pair<Coord, Coord> >* invalids_G,
		const std::map<Coord, int, coord_compare>* invalids_L);
	// const bgi::rtree<value, bgi::quadratic<8> >& GetInvalidPushes() const;
	void AddHallucinatedConstraint(const Coord &c);
	int InvalidPushCount(const Coord &c);

	void GetSuccs(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);
	bool CheckGoalCost(
		int state_id,
		std::vector<int>* succ_ids,
		std::vector<unsigned int>* costs);
	bool IsGoal(int state_id);

	unsigned int GetGoalHeuristic(int state_id);
	unsigned int GetConflictHeuristic(int state_id);
	unsigned int GetGoalHeuristic(const LatticeState& s);

	bool ConvertPath(
		const std::vector<int>& idpath);
private:

	Agent* m_agent = nullptr;

	std::vector<int> m_start_ids, m_goal_ids;
	STATES m_states, m_closed;

	std::list<std::shared_ptr<Constraint> > m_constraints;
	std::vector<std::pair<int, Trajectory> >* m_cbs_solution; // all agent trajectories
	int m_cbs_id, m_max_time;
	std::unordered_set<int> m_to_avoid;

	// std::set<Coord, coord_compare> m_invalid_pushes;
	typedef bg::model::point<int, 2, bg::cs::cartesian> point;
	typedef std::pair<point, int> value;
	bgi::rtree<value, bgi::quadratic<8> > m_invalid_pushes;

	// maps from coords to stateID
	typedef LatticeState StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

	int generateSuccessor(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const LatticeState* s1,
		const LatticeState* s2);
	unsigned int cost_gaussian_penalty(
		const LatticeState* s1,
		const LatticeState* s2);

	// PP
	int generateSuccessorPP(
		const LatticeState* parent,
		int dx, int dy,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs);
	unsigned int costPP(
		const LatticeState* s1,
		bool s1_outside_ngr,
		const LatticeState* s2);

	int conflictHeuristic(const LatticeState& state);
	bool goalConflict(const LatticeState& state);
	unsigned int checkNNCost(const Coord& c);

	LatticeState* getHashEntry(int state_id) const;
	int getHashEntry(
		const Coord& coord,
		const int& t);
	int reserveHashEntry();
	int createHashEntry(
		const Coord& coord,
		const State& state,
		const int& t,
		const int& hc);
	int getOrCreateState(
		const Coord& coord,
		const State& state,
		const int& t,
		const int& hc);
	int getOrCreateState(const LatticeState& s);
	int getOrCreateState(const Coord& p);
};

} // namespace clutter


#endif // AGENT_LATTICE_HPP
