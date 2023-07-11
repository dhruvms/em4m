#include <pushplan/sampling/rrt.hpp>
#include <pushplan/sampling/node.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/helpers.hpp>

#include <comms/ObjectsPoses.h>
#include <smpl/console/console.h>

namespace clutter {
namespace sampling {

RRT::RRT()
:
SamplingPlanner(),
m_N(10000), m_steps(50),
m_gbias(0.05), m_gthresh(DEG5/5),
m_timeout(120.0)
{}

RRT::RRT(int samples, int steps, double gbias, double gthresh, double timeout)
:
SamplingPlanner(),
m_N(samples), m_steps(steps),
m_gbias(gbias), m_gthresh(gthresh),
m_timeout(timeout)
{}

bool RRT::Solve()
{
	if (!SamplingPlanner::Solve())
	{
		SMPL_ERROR("Planner not setup!");
		return false;
	}

	m_stats["first_goal"] = -1.0;
	m_stats["sims"] = 0.0;
	m_stats["sim_time"] = 0.0;
	m_stats["soln_cost"] = 0.0;
	m_stats["first_soln_time"] = 0.0;
	m_stats["goal_samples"] = 0.0;
	m_stats["random_samples"] = 0.0;
	m_stats["goal_nodes"] = 0.0;

	addNode(m_start, m_start_v);

	double start_time = GetTime();
	for (int i = 0; i < m_N; ++i)
	{
		// get random sample
		smpl::RobotState qrand;
		bool try_goal = false;
		if (m_distD(m_rng) < m_gbias)
		{
			try_goal = true;
			m_goal_fn(qrand);
			++m_stats["goal_samples"];
		}
		else
		{
			m_robot->GetRandomState(qrand);
			++m_stats["random_samples"];
		}

		Vertex_t tree_v;
		std::uint32_t result;
		if (!extend(qrand, tree_v, result))
		{
			if (result == 10000) {
				SMPL_DEBUG("Failed to selectVertex from tree!");
			}
			else
			{
				if (result == 10001) {
					SMPL_DEBUG("Failed to SetScene for xnear!");
				}
				else if (result == 0) {
					SMPL_DEBUG("SteerAction computation failed!");
				}
				else if (result == 100 || result == 101) {
					SMPL_DEBUG("SteerAction failed in simulation!");
				}
			}
		}
		else
		{
			Node* xnew = m_G[tree_v];
			if ((try_goal && result == 1000) || poseWithinTolerance(xnew->robot_state(), 0.01, m_gthresh))
			{
				if (checkGoalNode(xnew))
				{
					xnew->set_goal_node();
					++m_goal_nodes;
					SMPL_DEBUG("Added a node close to the goal to the tree!");
					if (m_stats["first_goal"] < 0)
					{
						m_stats["first_goal"] = i;
						m_stats["first_soln_time"] = GetTime() - start_time;
					}

				}
			}

			if (result == 5 || result == 6) {
				SMPL_DEBUG("SteerAction added to tree without simulation!");
			}
			else if (result == 9 || result == 10) {
				SMPL_DEBUG("SteerAction added to tree after simulation!");
			}
		}

		if (GetTime() - start_time > m_timeout) {
			break;
		}
	}

	m_stats["vertices"] = (double)boost::num_vertices(m_G);
	m_stats["plan_time"] = GetTime() - start_time;
	m_stats["goal_nodes"] = m_goal_nodes;

	SMPL_INFO("Goal samples: %f, Random samples: %f | Goal nodes: %d", m_stats["goal_samples"], m_stats["random_samples"], m_goal_nodes);
	return m_goal_nodes > 0;
}

bool RRT::ExtractPath(std::vector<smpl::RobotState>& path)
{
	path.clear();

	// get tree vertex closest to goal state
	Graph_t::vertex_iterator vit_begin, vit_end;
	std::tie(vit_begin, vit_end) = boost::vertices(m_G);

	double best_dist = std::numeric_limits<double>::infinity();
	Vertex_t current_v, parent_v;

	if (boost::num_vertices(m_G) == 0) {
		SMPL_ERROR("Graph empty!");
		return false;
	}
	for (Graph_t::vertex_iterator it = vit_begin; it != vit_end; ++it)
	{
		double dist = poseGoalDistance(m_G[*it]->robot_state());
		if (dist < best_dist)
		{
			best_dist = dist;
			current_v = *it;
		}
	}
	SMPL_INFO("Extracting path to node with (best) goal distance = %f", best_dist);

	// get path
	path.insert(path.begin(), m_G[current_v]->robot_state());
	while (current_v != m_start_v)
	{
		if (boost::in_degree(current_v, m_G) != 1)
		{
			SMPL_ERROR("Found an RRT node with more than one parent!");
			return false;
		}

		Graph_t::in_edge_iterator eit_begin, eit_end;
		std::tie(eit_begin, eit_end) = boost::in_edges(current_v, m_G);
		parent_v = boost::source(*eit_begin, m_G);
		path.insert(path.begin(), m_G[parent_v]->robot_state());
		current_v = parent_v;
	}

	for (std::size_t i = 1; i < path.size(); ++i) {
		m_stats["soln_cost"] += configDistance(path[i-1], path[i]);
	}

	return true;
}

bool RRT::ExtractTraj(trajectory_msgs::JointTrajectory& exec_traj)
{
	std::vector<smpl::RobotState> path;
	ExtractPath(path);

	Trajectory path_traj;
	for (const auto& wp : path) {
		LatticeState s;
		s.state = wp;
		path_traj.push_back(std::move(s));
	}

	m_robot->ConvertTraj(path_traj, exec_traj);
}

bool RRT::extend(const smpl::RobotState& sample, Vertex_t& new_v, std::uint32_t& result)
{
	Node* xnear = nullptr;
	Vertex_t near_v;
	if (selectVertex(sample, near_v)) {
		xnear = m_G[near_v];
	}
	else
	{
		result = 0x0002710; // 10000
		return false;
	}

	if (configDistance(sample, xnear->robot_state()) < m_gthresh) {
		result = 0x000003E8; // 1000 if sampled config too near to tree node
		return true;
	}

	smpl::RobotState qnew;
	comms::ObjectsPoses qnew_objs;

	double steer_time = GetTime();
	bool steered = steer(sample, xnear, qnew, qnew_objs, result);
	steer_time = GetTime() - steer_time;

	if (steered)
	{
		// successful steer
		Node* xnew = new Node(qnew, qnew_objs, xnear);
		addNode(xnew, new_v);
		addEdge(near_v, new_v);
	}

	if (result >= 9 && result <= 101)
	{
		++m_stats["sims"];
		m_stats["sim_time"] += steer_time;
	}

	return steered;
}

bool RRT::selectVertex(const smpl::RobotState& qrand, Vertex_t& nearest)
{
	if (m_rtree.empty()) {
		return false;
	}

	std::vector<value> nn;
	// hax?
	point query_p;
	bg::set<0>(query_p, qrand.at(0));
	bg::set<1>(query_p, qrand.at(1));
	bg::set<2>(query_p, qrand.at(2));
	bg::set<3>(query_p, qrand.at(3));
	bg::set<4>(query_p, qrand.at(4));
	bg::set<5>(query_p, qrand.at(5));
	bg::set<6>(query_p, qrand.at(6));

	m_rtree.query(bgi::nearest(query_p, 1), std::back_inserter(nn));

	nearest = nn.front().second;
	return true;
}

bool RRT::steer(
	const smpl::RobotState& qrand,
	Node* xnear,
	smpl::RobotState& qnew,
	comms::ObjectsPoses& qnew_objs,
	std::uint32_t& result)
{
	if (!m_robot->SetScene(xnear->objects(), xnear->robot_state()))
	{
		result = 0x0002711;
		return false;
	}

	double frac;
	return m_robot->SteerAction(
			qrand, m_steps,
			xnear->robot_state(), xnear->objects(),
			qnew, qnew_objs, frac, result);
}

bool RRT::checkGoalNode(Node* node)
{
	if (!m_robot->SetScene(node->objects(), node->robot_state())) {
		return false;
	}

	return m_robot->CheckGraspTraj(node->robot_state(), node->objects());
}

} // namespace sampling
} // namespace clutter
