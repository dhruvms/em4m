#include <pushplan/agents/robot.hpp>
#include <pushplan/agents/agent.hpp>
#include <pushplan/search/cbs.hpp>
#include <pushplan/utils/helpers.hpp>
#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/utils/constants.hpp>

#include <smpl/console/console.h>

#include <ctime>

namespace clutter
{

CBS::CBS() :
m_ct_generated(0), m_ct_deadends(0), m_ct_expanded(0), m_ll_expanded(0), m_time_limit(30.0), m_soln_lb(0), m_wf(2)
{
	m_robot = nullptr;
	m_objs.clear();
	m_num_agents = 0;
	m_paths.clear();
	m_min_fs.clear();
	m_solved = false;
}

CBS::CBS(const std::shared_ptr<Robot>& r, const std::vector<std::shared_ptr<Agent> >& objs,
	int scene_id) :
m_ct_generated(0), m_ct_deadends(0), m_ct_expanded(0), m_ll_expanded(0), m_time_limit(30.0), m_scene_id(scene_id), m_soln_lb(0), m_wf(2)
{
	m_robot = r;
	m_objs = objs;
	m_num_agents = (int)m_objs.size();
	m_paths.resize(m_num_agents, nullptr);
	m_min_fs.resize(m_num_agents, 0);

	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		m_obj_id_to_idx[m_objs[i]->GetID()] = i;
		m_obj_idx_to_id[i] = m_objs[i]->GetID();
	}
	m_solved = false;
}

void CBS::AddObjects(const std::vector<std::shared_ptr<Agent> >& objs)
{
	m_objs.insert(m_objs.end(), objs.begin(), objs.end());
	m_num_agents = (int)m_objs.size();
	m_paths.resize(m_num_agents, nullptr);
	m_min_fs.resize(m_num_agents, 0);

	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		m_obj_id_to_idx[m_objs[i]->GetID()] = i;
		m_obj_idx_to_id[i] = m_objs[i]->GetID();
	}
}

bool CBS::Solve()
{
	m_search_time = 0.0;
	m_conflict_time = 0.0;
	m_ll_time = 0.0;
	double start_time = GetTime();
	if (!initialiseRoot()) {
		SMPL_ERROR("Failed to initialiseRoot");
		return false;
	}
	m_search_time += GetTime() - start_time; // add initialiseRoot time

	while (!m_OPEN.empty())
	{
		start_time = GetTime(); // reset clock with every loop iteration

		// select high level node
		if (m_OPEN.top()->fval() > m_soln_lb)
		{
			int f_thresh = m_wf * m_soln_lb;
			m_soln_lb = std::max(m_soln_lb, m_OPEN.top()->fval());
			int new_f_thresh = m_wf * m_soln_lb;
			for (auto& n : m_OPEN)
			{
				if (n->m_flowtime > f_thresh && n->m_flowtime <= new_f_thresh) {
					n->m_FOCAL_h = m_FOCAL.push(n);
				}
			}
		}
		auto next = m_FOCAL.top();
		m_FOCAL.pop();
		m_OPEN.erase(next->m_OPEN_h);
		// SMPL_INFO("SELECT");
		// next->PrintDebug();

		if (done(next))
		{
			// SMPL_INFO("CBS DONE!");
			m_search_time += GetTime() - start_time;
			return m_solved;
		}

		if (!next->m_h_computed)
		{
			next->computeH();
			// reinsert into OPEN because lowerbound changed?
			if (next->m_flowtime > m_wf * m_soln_lb)
			{
				next->m_OPEN_h = m_OPEN.push(next);
				continue;
			}
		}

		++m_ct_expanded;
		next->m_expand = m_ct_expanded;

		selectConflict(next);
		// SMPL_INFO("EXPAND");
		// next->PrintDebug();
		growConstraintTree(next);

		m_search_time += GetTime() - start_time;
	}

	SMPL_ERROR("CBS high-level OPEN is empty");
	return false;
}

void CBS::Reset()
{
	m_num_agents = 0;
	m_objs.clear();
	m_obj_id_to_idx.clear();
	m_obj_idx_to_id.clear();
	for (auto& c: m_paths) {
		c = nullptr;
	}
	m_paths.clear();
	m_min_fs.clear();

	m_ct_generated = 0;
	m_ct_deadends = 0;
	m_ct_expanded = 0;
	m_ll_expanded = 0;
	m_soln_lb = 0;
	m_solved = false;

	m_goal = nullptr;

	m_OPEN.clear();
	m_FOCAL.clear();
	for (auto& node: m_nodes)
	{
		if (node != nullptr) {
			delete node;
		}
	}
	m_nodes.clear();
}

void CBS::growConstraintTree(HighLevelNode* parent)
{
	// expand CT node
	HighLevelNode* child[2] = { new HighLevelNode() , new HighLevelNode() };
	addConstraints(parent, child[0], child[1]);
	for (int i = 0; i < 2; ++i)
	{
		if (updateChild(parent, child[i])) {
			parent->m_children.push_back(child[i]);
		}
		else {
			delete (child[i]);
			continue;
		}
	}
	parent->clear();
}

bool CBS::initialiseRoot()
{
	auto root = new HighLevelNode();
	root->m_g = 0;
	root->m_flowtime = 0;
	root->m_makespan = 0;
	root->m_depth = 0;
	root->m_parent = nullptr;
	root->m_children.clear();

	// Plan for robot
	int expands, min_f;
	double start_time = GetTime();
	// if (!m_robot->SatisfyPath(root, &m_paths[0], expands, min_f)) { // (CT node, path location)
	// 	++m_ct_deadends;
	// 	return false;
	// }
	// m_ll_time += GetTime() - start_time;
	// m_ll_expanded += expands;
	// m_min_fs[0] = min_f;
	// root->m_solution.emplace_back(m_robot->GetID(), *(m_paths[0]));
	// root->m_g += m_min_fs[0];
	// root->m_flowtime += m_paths[0]->size() - 1;
	// root->m_makespan = std::max(root->m_makespan, (int)m_paths[0]->size());

	// Plan for objects
	for (size_t i = 0; i < m_objs.size(); ++i)
	{
		start_time = GetTime();
		if (!m_objs[i]->SatisfyPath(root, &m_paths[i], expands, min_f)) {
			++m_ct_deadends;
			return false;
		}
		m_ll_time += GetTime() - start_time;
		m_ll_expanded += expands;
		m_min_fs[i] = min_f;
		root->m_solution.emplace_back(m_obj_idx_to_id[i], *(m_paths[i]));
		root->m_g += m_min_fs[i];
		root->m_flowtime += m_paths[i]->size() - 1;
		root->m_makespan = std::max(root->m_makespan, (int)m_paths[i]->size());
	}

	findConflicts(*root);

	root->m_h = 0;
	root->m_h_computed = false;
	root->updateDistanceToGo();

	pushNode(root);
	m_root = root;
	// SMPL_INFO("GENERATE");
	// root->PrintDebug();

	return true;
}

void CBS::pushNode(HighLevelNode* node)
{
	++m_ct_generated;
	node->m_generate = m_ct_generated;
	node->m_OPEN_h = m_OPEN.push(node);
	if (node->m_flowtime <= m_wf * m_soln_lb) {
		node->m_FOCAL_h = m_FOCAL.push(node);
	}
	m_nodes.push_back(node);
}

void CBS::findConflicts(HighLevelNode& node)
{
	// SMPL_INFO("findConflicts");
	double start_time = GetTime();
	if (node.m_parent == nullptr) // root node
	{
		// robot-object conflicts
		// for (size_t i = 0; i < m_objs.size(); ++i) {
		// 	findConflictsRobot(node, i);
		// }

		// object-object conflicts
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			for (size_t j = i+1; j < m_objs.size(); ++j) {
				findConflictsObjects(node, i, j);
			}
		}
	}
	else
	{
		copyRelevantConflicts(node);

		// if (node.m_replanned == 0) // robot
		// {
		// 	for (size_t i = 0; i < m_objs.size(); ++i) {
		// 		findConflictsRobot(node, i);
		// 	}
		// }
		// else
		{
			for (size_t i = 0; i < m_objs.size(); ++i)
			{
				if (m_obj_idx_to_id[i] != node.m_replanned) {
					continue;
				}

				// findConflictsRobot(node, i);
				for (size_t j = 0; j < m_objs.size(); ++j)
				{
					if (j == i) {
						continue;
					}
					findConflictsObjects(node, i, j);
				}
			}
		}
	}
	m_conflict_time += GetTime() - start_time;
}

void CBS::findConflictsRobot(HighLevelNode& curr, size_t oid)
{
	Trajectory* r_traj = &(curr.m_solution[0].second);
	Trajectory* a_traj = nullptr;
	for (auto& solution: curr.m_solution)
	{
		if (solution.first == m_obj_idx_to_id[oid])
		{
			a_traj = &(solution.second);
			break;
		}
	}

	if (a_traj == nullptr) {
		return;
	}

	int tmin = std::min(r_traj->size(), a_traj->size());
	for (int t = 0; t < tmin; ++t)
	{
		m_objs[oid]->UpdatePose(a_traj->at(t));
		if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->at(t), r_traj->at(t), t))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, r_traj->at(t), a_traj->at(t), true);
			curr.m_all_conflicts.push_back(conflict);
		}
	}

	if (r_traj->size() != a_traj->size())
	{
		bool robot_shorter = r_traj->size() < a_traj->size();
		auto* shorter = robot_shorter ? r_traj : a_traj;
		auto* longer = robot_shorter ? a_traj : r_traj;

		if (!robot_shorter)
		{
			m_objs[oid]->UpdatePose(a_traj->back());
			// add object to robot collision space
			std::vector<Object*> o;
			o.push_back(m_objs[oid]->GetObject());
			m_robot->ProcessObstacles(o);
		}

		for (int t = tmin; t < longer->size(); ++t)
		{
			if (robot_shorter)
			{
				m_objs[oid]->UpdatePose(longer->at(t));
				if (m_cc->RobotObjectCollision(m_objs[oid].get(), longer->at(t), shorter->back(), t))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, shorter->back(), longer->at(t), true);
					curr.m_all_conflicts.push_back(conflict);
				}
			}
			else
			{
				// if (m_robot->CheckCollision(longer->at(t), t))
				if (m_cc->RobotObjectCollision(m_objs[oid].get(), a_traj->back(), longer->at(t), t, false))
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					conflict->InitConflict(m_robot->GetID(), m_obj_idx_to_id[oid], t, longer->at(t), shorter->back(), true);
					curr.m_all_conflicts.push_back(conflict);
				}
			}
		}

		if (!robot_shorter)
		{
			// remove object from robot collision space
			std::vector<Object*> o;
			o.push_back(m_objs[oid]->GetObject());
			m_robot->ProcessObstacles(o, true);
		}
	}
}

void CBS::findConflictsObjects(HighLevelNode& curr, size_t o1, size_t o2)
{
	Trajectory* a1_traj = nullptr;
	Trajectory* a2_traj = nullptr;
	for (auto& solution: curr.m_solution)
	{
		if (solution.first == m_obj_idx_to_id[o1])
		{
			a1_traj = &(solution.second);
			continue;
		}
		else if (solution.first == m_obj_idx_to_id[o2])
		{
			a2_traj = &(solution.second);
			continue;
		}
		else {
			continue;
		}
	}

	if (a1_traj == nullptr || a2_traj == nullptr) {
		return;
	}

	int tmin = std::min(a1_traj->size(), a2_traj->size());
	for (int t = 1; t < tmin; ++t)
	{
		if (a1_traj->at(t).coord == a1_traj->at(0).coord && a2_traj->at(t).coord == a2_traj->at(0).coord) {
			continue;
		}

		m_objs[o1]->UpdatePose(a1_traj->at(t));
		m_objs[o2]->UpdatePose(a2_traj->at(t));
		if (m_cc->ObjectObjectCollision(m_objs[o1]->GetFCLObject(), m_objs[o2]->GetFCLObject()))
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->InitConflict(m_obj_idx_to_id[o1], m_obj_idx_to_id[o2], t, a1_traj->at(t), a2_traj->at(t), false);
			curr.m_all_conflicts.push_back(conflict);
			// SMPL_INFO("Conflict between %d (traj size %d) and %d (traj size %d) at time %d", m_obj_idx_to_id[o1], a1_traj->size(), m_obj_idx_to_id[o2], a2_traj->size(), t);
		}
	}

	if (a1_traj->size() != a2_traj->size())
	{
		LatticeState terminal;
		bool a1_shorter = a1_traj->size() < a2_traj->size();
		Agent* shorter = a1_shorter ? m_objs[o1].get() : m_objs[o2].get();
		Agent* longer = a1_shorter ? m_objs[o2].get() : m_objs[o1].get();
		auto* shorter_traj = a1_shorter ? a1_traj : a2_traj;
		auto* longer_traj = a1_shorter ? a2_traj : a1_traj;

		shorter->UpdatePose(shorter_traj->back());
		for (int t = tmin; t < longer_traj->size(); ++t)
		{
			if (shorter_traj->back().coord == shorter_traj->at(0).coord && longer_traj->at(t).coord == longer_traj->at(0).coord) {
				continue;
			}

			longer->UpdatePose(longer_traj->at(t));
			if (m_cc->ObjectObjectCollision(shorter->GetFCLObject(), longer->GetFCLObject()))
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				conflict->InitConflict(shorter->GetID(), longer->GetID(), t, shorter_traj->back(), longer_traj->at(t), false);
				curr.m_all_conflicts.push_back(conflict);
				// SMPL_INFO("Conflict between %d (traj size %d) and %d (traj size %d) at time %d", shorter->GetID(), shorter_traj->size(), longer->GetID(), longer_traj->size(), t);
			}
		}
	}
}

void CBS::copyRelevantConflicts(HighLevelNode& node) const
{
	for (auto& conflict : node.m_parent->m_conflicts)
	{
		if (conflict->m_a1 == node.m_replanned || conflict->m_a2 == node.m_replanned) {
			continue;
		}
		node.m_conflicts.push_back(conflict);
	}

	for (auto& conflict : node.m_parent->m_all_conflicts)
	{
		if (conflict->m_a1 == node.m_replanned || conflict->m_a2 == node.m_replanned) {
			continue;
		}
		node.m_all_conflicts.push_back(conflict);
	}
}

bool CBS::selectConflict(HighLevelNode* node) const
{
	if (node->m_conflicts.empty() && node->m_all_conflicts.empty())
	{
		SMPL_ERROR("Trying to expand CT node with no conflicts.");
		return false;
	}

	while (!node->m_all_conflicts.empty())
	{
		auto conflict = node->m_all_conflicts.front();
		node->m_all_conflicts.pop_front();
		conflict->SetPriority(node);
		node->m_conflicts.push_back(conflict);
	}

	if (!node->m_conflicts.empty())
	{
		std::unordered_map<int, std::shared_ptr<Conflict> > to_keep;
		std::list<std::shared_ptr<Conflict> > to_delete;
		for (const auto& conflict : node->m_conflicts)
		{
			int a1 = std::min(conflict->m_a1, conflict->m_a2), a2 = std::max(conflict->m_a1, conflict->m_a2);
			int key = a1 * m_num_agents + a2;
			auto p = to_keep.find(key);
			if (p == to_keep.end()) {
				to_keep[key] = conflict;
			}
			else if (*(p->second) < *conflict)
			{
				to_delete.push_back(p->second);
				to_keep[key] = conflict;
			}
			else {
				to_delete.push_back(conflict);
			}
		}

		for (const auto& conflict : to_delete) {
			node->m_conflicts.remove(conflict);
		}
	}

	std::shared_ptr<Conflict> selected;
	if (node->m_conflicts.empty() && node->m_all_conflicts.empty())
	{
		SMPL_WARN("CT node has no conflicts. What happens now?");
	}
	else if (!node->m_conflicts.empty())
	{
		selected = node->m_conflicts.back();
		for (const auto& conflict : node->m_conflicts)
		{
			if (*selected < *conflict) {
				selected = conflict;
			}
		}
	}
	else
	{
		selected = node->m_all_conflicts.back();
		for (const auto& conflict : node->m_all_conflicts)
		{
			if (*selected < *conflict) {
				selected = conflict;
			}
		}
	}

	if (selected)
	{
		node->m_conflict = selected;
		node->m_conflict->m_on = true;
	}
	else
	{
		node->m_conflict = std::make_shared<Conflict>();
		node->m_conflict->m_on = false;
	}

	return true;
}

void CBS::addConstraints(const HighLevelNode* curr, HighLevelNode* child1, HighLevelNode* child2) const
{
	// children inherit all constraints of parent node
	child1->m_constraints = curr->m_constraints;
	child2->m_constraints = curr->m_constraints;

	// children get one new constraint each
	child1->m_constraints.push_back(curr->m_conflict->m_c1);
	child2->m_constraints.push_back(curr->m_conflict->m_c2);
}

bool CBS::updateChild(HighLevelNode* parent, HighLevelNode* child)
{
	child->m_g = parent->m_g; // for now
	child->m_flowtime = parent->m_flowtime; // for now
	child->m_makespan = parent->m_makespan; // for now
	child->m_solution = parent->m_solution; // for now
	child->m_depth = parent->m_depth + 1;
	child->m_parent = parent;
	child->m_children.clear();

	// agent to be replanned for
	child->m_replanned = child->m_constraints.back()->m_me;
	int expands, min_f;

	// replan for agent
	bool recalc_makespan = true;
	double start_time;
	// if (child->m_replanned == 0)
	// {
	// 	child->m_g -= m_min_fs[0];
	// 	child->m_flowtime -= m_paths[0]->size() - 1;
	// 	if (child->m_makespan == m_paths[0]->size()) {
	// 		recalc_makespan = true;
	// 	}

	// 	start_time = GetTime();
	// 	if (!m_robot->SatisfyPath(child, &m_paths[0], expands, min_f)) {
	// 		++m_ct_deadends;
	// 		return false;
	// 	}
	// 	m_ll_time += GetTime() - start_time;
	// 	m_ll_expanded += expands;

	// 	// update solution in CT node
	// 	for (auto& solution : child->m_solution)
	// 	{
	// 		if (solution.first == m_robot->GetID()) {
	// 			solution.second = *(m_paths[0]);
	// 			break;
	// 		}
	// 	}

	// 	child->m_g += min_f;
	// 	child->m_flowtime += m_paths[0]->size() - 1;
	// 	m_min_fs[0] = min_f;
	// 	if (recalc_makespan) {
	// 		child->recalcMakespan();
	// 	}
	// 	else {
	// 		child->m_makespan = std::max(child->m_makespan, (int)m_paths[0]->size());
	// 	}
	// }
	// else
	{
		for (size_t i = 0; i < m_objs.size(); ++i)
		{
			if (m_obj_idx_to_id[i] != child->m_replanned) {
				continue;
			}

			child->m_g -= m_min_fs[i];
			child->m_flowtime -= m_paths[i]->size() - 1;
			if (child->m_makespan == m_paths[i]->size()) {
				recalc_makespan = true;
			}

			m_objs[i]->Init();
			start_time = GetTime();
			if (!m_objs[i]->SatisfyPath(child, &m_paths[i], expands, min_f))
			{
				++m_ct_deadends;
				return false;
			}
			m_ll_time += GetTime() - start_time;
			m_ll_expanded += expands;
			m_min_fs[i] = min_f;

			// update solution in CT node
			for (auto& solution : child->m_solution)
			{
				if (solution.first == m_obj_idx_to_id[i]) {
					solution.second = *(m_paths[i]);
					break;
				}
			}

			child->m_g += m_min_fs[i];
			child->m_flowtime += m_paths[i]->size() - 1;
			if (recalc_makespan) {
				child->recalcMakespan();
			}
			else {
				child->m_makespan = std::max(child->m_makespan, (int)m_paths[i]->size());
			}

			break;
		}
	}

	findConflicts(*child);

	child->m_h = std::max(0, int(parent->fval() - child->m_g)); // booooo
	child->m_h_computed = false;
	child->updateDistanceToGo();
	child->updateCostToGo();

	pushNode(child);
	// SMPL_INFO("GENERATE");
	// child->PrintDebug();
	return true;
}

bool CBS::done(HighLevelNode* node)
{
	if (node->m_conflicts.empty() && node->m_all_conflicts.empty())
	{
		ROS_WARN("CBS solved! HL Nodes expanded = %d", m_ct_expanded);
		m_solved = true;
		m_goal = node;
		m_soln_cost = m_goal->m_flowtime;

		for (const auto& solution : m_goal->m_solution) {
			m_objs[m_obj_id_to_idx[solution.first]]->SetSolveTraj(solution.second);
		}

		// if (!m_goal->m_priorities.Empty())
		// {
		// 	SMPL_WARN("Solution Priority DAG:");
		// 	auto G = m_goal->m_priorities.GetDAG();
		// 	for (const auto& parent: G) {
		// 		for (const auto& child: parent.second) {
		// 			SMPL_WARN("\t%d -> %d", parent.first, child);
		// 		}
		// 	}
		// }
		// else {
		// 	SMPL_WARN("Solution has empty Priority DAG!");
		// }

		return true;
	}

	if (m_search_time > m_time_limit)
	{
		ROS_WARN("CBS search time exceeded! HL Nodes expanded = %d", m_ct_expanded);
		m_solved = false;
		m_soln_cost = -1;
		return true;
	}

	// not done
	return false;
}

void CBS::SaveStats()
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/?.csv";

	std::string algoname;
	switch (ALGO)
	{
		case MAPFAlgo::ECBS:
		{
			algoname = "ECBS";
			break;
		}
		case MAPFAlgo::CBSWP:
		{
			algoname = "CBSwP";
			break;
		}
		case MAPFAlgo::PBS:
		{
			algoname = "PBS";
			break;
		}
		case MAPFAlgo::VCBS:
		default:
		{
			algoname = "CBS";
			break;
		}
	}
	found = filename.find_last_of("?");
	filename.insert(found, algoname);
	found = filename.find_last_of("?");
	filename.erase(found, 1);

	bool exists = FileExists(filename);
	std::ofstream STATS;
	STATS.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		STATS << "UID,"
				<< "Solved?,SolveTime,SolutionCost,"
				<< "HLGenerated,HLDeadends,HLExpanded,"
				<< "LLExpanded,LLTime,ConflictsTime\n";
	}

	STATS << m_scene_id << ','
			<< (int)m_solved << ',' << m_search_time << ',' << m_soln_cost << ','
			<< m_ct_generated << ',' << m_ct_deadends << ',' << m_ct_expanded << ','
			<< m_ll_expanded << ',' << m_ll_time << ',' << m_conflict_time << '\n';
	STATS.close();
}

bool CBS::UpdateStats(std::map<std::string, double>& stats)
{
	if (stats.empty())
	{
		stats["calls"] = 1;
		stats["solved"] = (int)m_solved;
		stats["search_time"] = m_search_time;
		stats["ct_nodes"] = m_ct_generated;
		stats["ct_deadends"] = m_ct_deadends;
		stats["ct_expanded"] = m_ct_expanded;
		stats["ll_time"] = m_ll_time;
		stats["conflict_time"] = m_conflict_time;
		stats["makespan"] = m_solved ? m_goal->m_makespan : -1;
		stats["flowtime"] = m_solved ? m_goal->m_flowtime : -1;

		return true;
	}

	stats["calls"] += 1;
	stats["solved"] += (int)m_solved;
	stats["search_time"] += m_search_time;
	stats["ct_nodes"] += m_ct_generated;
	stats["ct_deadends"] += m_ct_deadends;
	stats["ct_expanded"] += m_ct_expanded;
	stats["ll_time"] += m_ll_time;
	stats["conflict_time"] += m_conflict_time;

	return true;
}

} // namespace clutter
