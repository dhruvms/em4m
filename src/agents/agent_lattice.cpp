#include <pushplan/agents/agent_lattice.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/utils/discretisation.hpp>
#include <pushplan/utils/geometry.hpp>

#include <boost/foreach.hpp>

auto std::hash<clutter::LatticeState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.begin() + 2));
	boost::hash_combine(seed, s.t);
	return seed;
}

namespace clutter
{

void AgentLattice::init(Agent* agent)
{
	m_agent = agent;
}

void AgentLattice::reset()
{
	// reset everything for new CBS calls
	for (LatticeState* s : m_states)
	{
		if (s != nullptr)
		{
			delete s;
			s = nullptr;
		}
	}
	m_states.clear();
	m_closed.clear();

	m_start_ids.clear();
	m_goal_ids.clear();
	m_state_to_id.clear();
}

int AgentLattice::PushStart(const LatticeState& s)
{
	int start_id = getOrCreateState(s);
	m_start_ids.push_back(start_id);
	return start_id;
}

int AgentLattice::PushGoal(const Coord& p)
{
	int goal_id = getOrCreateState(p);
	m_goal_ids.push_back(goal_id);

	Coord pseudogoal = { -99, -99 };
	goal_id = getOrCreateState(pseudogoal);
	m_goal_ids.push_back(goal_id);

	return goal_id;
}

void AgentLattice::SetCTNode(HighLevelNode* ct_node)
{
	m_constraints.clear();
	for (auto& constraint : ct_node->m_constraints)
	{
		if (constraint->m_me == ct_node->m_replanned) {
			m_constraints.push_back(constraint);
		}
	}

	m_cbs_solution = &(ct_node->m_solution);
	m_cbs_id = ct_node->m_replanned;
	m_max_time = ct_node->m_makespan;
}

void AgentLattice::AvoidAgents(const std::unordered_set<int>& to_avoid)
{
	m_to_avoid.clear();
	m_to_avoid = to_avoid;
}

void AgentLattice::ResetInvalidPushes(
	const std::vector<std::pair<Coord, Coord> >* invalids_G,
	const std::map<Coord, int, coord_compare>* invalids_L)
{
	m_invalid_pushes.clear();
	point p;
	for (size_t i = 0; i < invalids_G->size(); ++i)
	{
		if (invalids_G->at(i).first == m_agent->InitState().coord)
		{
			bg::set<0>(p, invalids_G->at(i).second.at(0));
			bg::set<1>(p, invalids_G->at(i).second.at(1));
			m_invalid_pushes.insert(std::make_pair(p, SAMPLES * 2));
			// m_invalid_pushes.insert(invalids_G->at(i).second);
		}
	}

	if (invalids_L != nullptr)
	{
		for (auto it = invalids_L->cbegin(); it != invalids_L->cend(); ++it)
		{
			bg::set<0>(p, it->first.at(0));
			bg::set<1>(p, it->first.at(1));
			m_invalid_pushes.insert(std::make_pair(p, it->second));
			// m_invalid_pushes.insert(it->first);
		}
	}
}

// const bgi::rtree<value, bgi::quadratic<8> >& AgentLattice::GetInvalidPushes() const
// {
// 	return m_invalid_pushes;
// }

void AgentLattice::AddHallucinatedConstraint(const Coord &c)
{
	point p(c.at(0), c.at(1));
	m_invalid_pushes.insert(std::make_pair(p, 1));
	// m_invalid_pushes.insert(c);
}

int AgentLattice::InvalidPushCount(const Coord &c)
{
	point p(c.at(0), c.at(1));
	int count = m_invalid_pushes.count(p);

	if (count == 0) {
		return count;
	}

	std::vector<value> nns;
	m_invalid_pushes.query(bgi::nearest(p, count), std::back_inserter(nns));
	count = 0;
	BOOST_FOREACH(const value &v, nns) {
		count = std::max(count, v.second);
	}
	return count;
	// const auto it = m_invalid_pushes.find(c);
	// if (it != m_invalid_pushes.end()) {
	// 	return 1;
	// }
	// else {
	// 	return 0;
	// }
}

void AgentLattice::GetSuccs(
	int state_id,
	std::vector<int>* succ_ids,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succ_ids->clear();
	costs->clear();

	LatticeState* parent = getHashEntry(state_id);
	assert(parent);
	m_closed.push_back(parent);

	// if (IsGoal(state_id)) {
	// 	SMPL_WARN("We are expanding the goal state (???)");
	// 	return;
	// }

	for (int dx = -1; dx <= 1; ++dx)
	{
		for (int dy = -1; dy <= 1; ++dy)
		{
			// ignore ordinal directions for 4-connected grid
			if (GRID == 4 && std::abs(dx * dy) == 1) {
				continue;
			}

			if (m_agent->PP()) {
				generateSuccessorPP(parent, dx, dy, succ_ids, costs);
			}
			else {
				generateSuccessor(parent, dx, dy, succ_ids, costs);
			}
		}
	}
}

bool AgentLattice::CheckGoalCost(
	int state_id,
	std::vector<int>* succ_ids,
	std::vector<unsigned int>* costs)
{
	if (!this->IsGoal(state_id)) {
		return false;
	}

	if (m_invalid_pushes.empty()) // state is a valid goal, and there are no invalid goals
	{
		succ_ids->push_back(m_goal_ids.back()); // pseudogoal
		costs->push_back(1);
	}
	else
	{
		// need to compute pseudo-edge cost for valid goal state
		LatticeState* s = getHashEntry(state_id);
		unsigned int cost = checkNNCost(s->coord);
		succ_ids->push_back(m_goal_ids.back()); // pseudogoal
		costs->push_back(cost);
	}

	return true;
}

// As long as I am not allowed to be in this location at some later time,
// I have not reached a valid goal state
// Conversely, if I can remain in this location (per existing constraints),
// I am at a valid goal state (since states in collision with immovable obstacles
// or out of bounds will never enter OPEN)
bool AgentLattice::IsGoal(int state_id)
{
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	point p(s->coord.at(0), s->coord.at(1));
	int count = m_invalid_pushes.count(p);
	if (count > 0) {
		return false;
	}

	bool constrained = false, conflict = false, ngr = false;
	ngr = m_agent->OutsideNGR(*s);
	if (m_agent->PP())
	{
		if (!ngr) {
			return false;
		}

		return !m_agent->PrioritisedCollisionCheck(*s, true);
	}

	if (ngr)
	{
		for (const auto& constraint : m_constraints)
		{
			if (constraint->m_q.coord == s->coord) {
				if (constraint->m_time >= s->t) {
					constrained = true;
					break;
				}
			}
		}

		// return !constrained;
		if (!constrained)
		{
			bool conflict = goalConflict(*s);

			// for forward search goal must valid for all future time
			// if (!conflict) {
			// 	m_agent->VisualiseState(*s, "valid_goal", 147);
			// 	SMPL_WARN("Goal visualised!");
			// }
			return !conflict;
		}

		// for forward search goal must not be constrained
		return false;
	}

	// for forward search goal must be outside NGR
	return false;
}

unsigned int AgentLattice::GetGoalHeuristic(int state_id)
{
	return 0;
	if (state_id == m_goal_ids.back() || m_agent->PP()) {
		return 0;
	}
	// TODO: RRA* informed backwards Dijkstra's heuristic
	// TODO: Try penalising distance to shelf edge?
	assert(state_id >= 0);
	LatticeState* s = getHashEntry(state_id);
	assert(s);

	return ManhattanDist(s->coord, m_agent->Goal());
}

unsigned int AgentLattice::GetConflictHeuristic(int state_id)
{
	assert(state_id >= 0);
	if (state_id == m_goal_ids.back()) {
		return 0;
	}

	LatticeState* s = getHashEntry(state_id);
	assert(s);
	return s->hc;
}

unsigned int AgentLattice::GetGoalHeuristic(const LatticeState& s)
{
	return 0;
	if (m_agent->PP()) {
		return 0;
	}
	// TODO: RRA* informed backwards Dijkstra's heuristic
	return ManhattanDist(s.coord, m_agent->Goal());
}

int AgentLattice::generateSuccessor(
	const LatticeState* parent,
	int dx, int dy,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	LatticeState child;
	child.t = parent->t + 1;
	child.hc = parent->hc;
	child.coord = parent->coord;
	child.coord.at(0) += dx;
	child.coord.at(1) += dy;

	child.state.clear();
	child.state.push_back(DiscretisationManager::DiscXToContX(child.coord.at(0)));
	child.state.push_back(DiscretisationManager::DiscYToContY(child.coord.at(1)));
	child.state.insert(child.state.end(), parent->state.begin() + 2, parent->state.end());

	// m_agent->VisualiseState(child, "expansion");

	m_agent->UpdatePose(child);
	if (m_agent->OutOfBounds(child) || m_agent->ImmovableCollision()) {
		return -1;
	}

	for (const auto& constraint : m_constraints)
	{
		if (child.t == constraint->m_time)
		{
			// Conflict type 1: robot-object conflict
			if (constraint->m_me == constraint->m_other)
			{
	 		// 	if (m_cbs_solution->at(0).second.size() <= constraint->m_time)
				// {
				// 	// This should never happen - the constraint would only have existed
				// 	// if this object and the robot had a conflict at that time
				// 	SMPL_WARN("How did this robot-object conflict happen with a small robot traj?");
				// 	continue;
				// }

				// // successor is invalid if I collide in state 'child'
				// // with the robot configuration at the same time
				// if (m_cc->RobotObjectCollision(
				// 			this, child,
				// 			m_cbs_solution->at(0).second.at(constraint->m_time), constraint->m_time))
				// {
				// 	return -1;
				// }
			}
			// Conflict type 2: object-object conflict
			else
			{
				// successor is invalid if I collide in state 'child'
				// with the constraint->m_other object in state constraint->m_q
				if (m_agent->ObjectObjectCollision(constraint->m_other, constraint->m_q)) {
					return -1;
				}
			}
		}
	}

	if (!m_to_avoid.empty())
	{
		std::vector<LatticeState> other_poses;
		std::vector<int> other_ids;

		for (const auto& agent_traj: *m_cbs_solution)
		{
			if (agent_traj.first == m_cbs_id || agent_traj.first == 0) {
				continue;
			}
			if (m_to_avoid.find(agent_traj.first) == m_to_avoid.end()) {
				continue;
			}

			other_ids.push_back(agent_traj.first);
			if (agent_traj.second.size() <= child.t) {
				other_poses.push_back(agent_traj.second.back());
			}
			else {
				other_poses.push_back(agent_traj.second.at(child.t));
			}
		}
		if (m_agent->ObjectObjectsCollision(other_ids, other_poses)) {
			return -1;
		}
	}

	child.hc += conflictHeuristic(child);

	int succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	// For ECBS (the state of the object is collision checked with the state of all agents
	// and the resulting number of collisions is used as part of the cost function
	// therefore not a hard constraint (like constraints).)

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));
	// costs->push_back(cost_gaussian_penalty(parent, successor));

	return succ_state_id;
}

int AgentLattice::generateSuccessorPP(
	const LatticeState* parent,
	int dx, int dy,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	LatticeState child;
	child.t = parent->t + 1;

	child.coord = parent->coord;
	child.coord.at(0) += dx;
	child.coord.at(1) += dy;

	child.state.clear();
	child.state.push_back(DiscretisationManager::DiscXToContX(child.coord.at(0)));
	child.state.push_back(DiscretisationManager::DiscYToContY(child.coord.at(1)));
	child.state.insert(child.state.end(), parent->state.begin() + 2, parent->state.end());

	m_agent->UpdatePose(child);
	if (m_agent->OutOfBounds(child) || m_agent->ImmovableCollision()) {
		return -1;
	}

	if (m_agent->PrioritisedCollisionCheck(child)) {
		return -1;
	}

	int succ_state_id = getOrCreateState(child);
	LatticeState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(costPP(parent, true, successor));

	return succ_state_id;
}

unsigned int AgentLattice::cost(
	const LatticeState* s1,
	const LatticeState* s2)
{
	double dist = ManhattanDist(s1->coord, s2->coord);
	dist = dist == 0.0 ? 1.0 : dist;
	return dist;
}

unsigned int AgentLattice::cost_gaussian_penalty(
	const LatticeState* s1,
	const LatticeState* s2)
{
	double obs_cost = m_agent->ObstacleGaussianCost(s2->state[0], s2->state[1]);
	return cost(s1, s2) + obs_cost;
}

unsigned int AgentLattice::costPP(
	const LatticeState* s1,
	bool s1_outside_ngr,
	const LatticeState* s2)
{
	return std::max(1.0, ManhattanDist(s1->coord, s2->coord));
}

int AgentLattice::conflictHeuristic(const LatticeState& state)
{
	int hc = 0;
	switch (LLHC)
	{
		case LowLevelConflictHeuristic::ZERO:
		{
			hc = 0;
			break;
		}
		case LowLevelConflictHeuristic::BINARY:
		{
			std::vector<LatticeState> other_poses;
			std::vector<int> other_ids;

			for (const auto& agent_traj: *m_cbs_solution)
			{
				if (agent_traj.first == m_cbs_id || agent_traj.first == 0) {
					continue;
				}

				other_ids.push_back(agent_traj.first);
				if (agent_traj.second.size() <= state.t) {
					other_poses.push_back(agent_traj.second.back());
				}
				else {
					other_poses.push_back(agent_traj.second.at(state.t));
				}
			}
			bool conflict = m_agent->ObjectObjectsCollision(other_ids, other_poses);

			// if (!conflict)
			// {
			// 	if (m_cbs_solution->at(0).second.size() <= state.t) {
			// 		conflict = conflict ||
			// 			m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.back(), state.t);
			// 	}
			// 	else {
			// 		conflict = conflict ||
			// 			m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.at(state.t), state.t);
			// 	}
			// }

			hc = (int)conflict;
			break;
		}
		case LowLevelConflictHeuristic::COUNT:
		{
			LatticeState other_pose;
			for (const auto& other_agent: *m_cbs_solution)
			{
				if (other_agent.first == m_cbs_id || other_agent.first == 0) {
					continue;
				}

				// other agent trajectory is shorter than current state's time
				// so we only collision check against the last state along the
				// other agent trajectory
				if (other_agent.second.size() <= state.t) {
					other_pose = other_agent.second.back();
				}
				else {
					// if the other agent has a trajectory longer than current state's time
					// we collision check against the current state's time
					other_pose = other_agent.second.at(state.t);
				}
				if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
					++hc;
				}
			}

			// // same logic for robot
			// if (m_cbs_solution->at(0).second.size() <= state.t) {
			// 	other_pose = m_cbs_solution->at(0).second.back();
			// }
			// else {
			// 	other_pose = m_cbs_solution->at(0).second.at(state.t);
			// }
			// if (m_cc->RobotObjectCollision(this, state, other_pose, state.t)) {
			// 	++hc;
			// }
			break;
		}
		case LowLevelConflictHeuristic::OURS:
		{
			hc = (int)std::pow(10, 1 - (m_agent->ObsDist(state.state[0], state.state[1])/0.15));
			break;
		}
		default:
		{
			SMPL_ERROR("Unknown conflict heuristic type!");
		}
	}

	return hc;
}

bool AgentLattice::goalConflict(const LatticeState& state)
{
	bool at_start = state == m_agent->InitState();
	m_agent->UpdatePose(state);

	LatticeState other_pose;
	for (const auto& other_agent: *m_cbs_solution)
	{
		if (other_agent.first == m_cbs_id || other_agent.first == 0) {
			continue;
		}

		// other agent trajectory is shorter than current state's time
		// so we only collision check against the last state along the
		// other agent trajectory
		if (other_agent.second.size() <= state.t)
		{
			other_pose = other_agent.second.back();
			if (at_start && other_pose == other_agent.second.front()) {
				continue;
			}

			if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
				return true;
			}
		}
		else
		{
			// if the other agent has a trajectory longer than current state's time
			// we collision check against all states in that trajectory beyond
			// the current state's time
			for (int t = state.t; t < (int)other_agent.second.size(); ++t)
			{
				other_pose = other_agent.second.at(t);
				if (at_start && other_pose == other_agent.second.front()) {
					continue;
				}

				if (m_agent->ObjectObjectCollision(other_agent.first, other_pose)) {
					return true;
				}
			}
		}
	}

	// // same logic for robot
	// if (m_cbs_solution->at(0).second.size() <= state.t)
	// {
	// 	if (m_cc->RobotObjectCollision(this, state, m_cbs_solution->at(0).second.back(), state.t)) {
	// 		return true;
	// 	}
	// }
	// else
	// {
	// 	for (int t = state.t; t < (int)m_cbs_solution->at(0).second.size(); ++t)
	// 	{
	// 		other_pose = m_cbs_solution->at(0).second.at(t);
	// 		if (m_cc->RobotObjectCollision(this, state, other_pose, state.t)) {
	// 			return true;
	// 		}
	// 	}
	// }

	return false;
}

unsigned int AgentLattice::checkNNCost(const Coord& c)
{
	std::vector<value> nns;
	point query(c.at(0), c.at(1));
	bg::model::box<point> b(point(c.at(0) - 3, c.at(1) - 3), point(c.at(0) + 3, c.at(1) + 3));
	m_invalid_pushes.query(bgi::within(b), std::back_inserter(nns));

	if (nns.empty()) {
		return 1;
	}

	double cost = 0.0;
	BOOST_FOREACH(const value &v, nns)
	{
		double d = bg::distance(v.first, query);
		if (d < 1) {
			cost += SAMPLES * 2; // max penalty if query is known invalid goal
		}
		else {
			cost += v.second/std::ceil(d);
		}
	}
	// // exponential between (1, 2) and (5, 10)
	// double cost = 25.831 * std::exp(-0.256 * dist);
	// cost = std::max(std::min(20.0, cost), 2.0);
	return std::ceil(cost);
}

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
LatticeState* AgentLattice::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int AgentLattice::getHashEntry(
	const Coord& coord,
	const int& t)
{
	LatticeState s;
	s.coord = coord;
	s.t = t;

	auto sit = m_state_to_id.find(&s);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int AgentLattice::reserveHashEntry()
{
	LatticeState* entry = new LatticeState;
	int state_id = (int)m_states.size();

	// map state id -> state
	m_states.push_back(entry);

	// // map planner state -> graph state
	// int* pinds = new int[NUMOFINDICES_STATEID2IND];
	// std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
	// StateID2IndexMapping.push_back(pinds);

	return state_id;
}

int AgentLattice::createHashEntry(
	const Coord& coord,
	const State& state,
	const int& t,
	const int& hc)
{
	int state_id = reserveHashEntry();
	LatticeState* entry = getHashEntry(state_id);

	entry->coord = coord;
	entry->state = state;
	entry->t = t;
	entry->hc = hc;

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

int AgentLattice::getOrCreateState(
	const Coord& coord,
	const State& state,
	const int& t,
	const int& hc)
{
	int state_id = getHashEntry(coord, t);
	if (state_id < 0) {
		state_id = createHashEntry(coord, state, t, hc);
	}
	return state_id;
}

int AgentLattice::getOrCreateState(const LatticeState& s)
{
	return getOrCreateState(s.coord, s.state, s.t, s.hc);
}

int AgentLattice::getOrCreateState(const Coord& p)
{
	State temp;
	return getOrCreateState(p, temp, -1, -1);
}

bool AgentLattice::ConvertPath(
	const std::vector<int>& idpath)
{
	Trajectory* opath = m_agent->SolveTraj(); // vector of LatticeState
	opath->clear();

	if (idpath.empty()) {
		return true;
	}

	if (std::find(m_goal_ids.begin(), m_goal_ids.end(), idpath[0]) != m_goal_ids.end())
	{
		SMPL_ERROR("Cannot extract a non-trivial path starting from the goal state");
		return false;
	}

	LatticeState state;
	// grab the first point
	auto* entry = getHashEntry(idpath[0]);
	if (!entry)
	{
		SMPL_ERROR("Failed to get state entry for state %d", idpath[0]);
		return false;
	}
	state = *entry;
	opath->push_back(state);

	// grab the rest of the points
	for (size_t i = 1; i < idpath.size(); ++i)
	{
		auto prev_id = idpath[i - 1];
		auto curr_id = idpath[i];

		if (prev_id == m_goal_ids.back())
		{
			SMPL_ERROR("Cannot determine goal state predecessor state during path extraction");
			return false;
		}

		auto* entry = getHashEntry(curr_id);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry state %d", curr_id);
			return false;
		}
		state = *entry;
		opath->push_back(state);
	}
	return true;
}

} // namespace clutter
