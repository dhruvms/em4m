#include <pushplan/agents/agent.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/search/cbs_nodes.hpp>
#include <pushplan/search/conflicts.hpp>
#include <pushplan/search/wastar.hpp>
#include <pushplan/search/focal.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/discretisation.hpp>
#include <pushplan/utils/geometry.hpp>

#include <smpl/console/console.h>
#include <smpl/ros/propagation_distance_field.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/debug/marker_conversions.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/marker.h>
#include <sbpl_collision_checking/collision_operations.h>
#include <leatherman/viz.h>

#include <iostream>
#include <algorithm>

namespace clutter
{

bool Agent::ResetObject()
{
	LatticeState s;
	s.state = { 	m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z,
						m_obj_desc.o_roll, m_obj_desc.o_pitch, m_obj_desc.o_yaw };
	UpdatePose(s);
}

bool Agent::SetObjectPose(
	const std::vector<double>& xyz,
	const std::vector<double>& rpy)
{
	m_obj_desc.o_x = xyz.at(0);
	m_obj_desc.o_y = xyz.at(1);
	m_obj_desc.o_z = xyz.at(2);

	m_obj_desc.o_roll = rpy.at(0);
	m_obj_desc.o_pitch = rpy.at(1);
	m_obj_desc.o_yaw = rpy.at(2);

	m_obj.desc = m_obj_desc;

	ResetObject();
}

bool Agent::SetObjectPose(const ContPose& pose)
{
	m_obj_desc.o_x = pose.x();
	m_obj_desc.o_y = pose.y();
	m_obj_desc.o_z = pose.z();

	m_obj_desc.o_roll = pose.roll();
	m_obj_desc.o_pitch = pose.pitch();
	m_obj_desc.o_yaw = pose.yaw();

	m_obj.desc = m_obj_desc;

	ResetObject();
}

bool Agent::Init()
{
	m_init.t = 0;
	m_init.hc = 0;
	m_init.state.clear();
	m_init.state = { 	m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z,
						m_obj_desc.o_roll, m_obj_desc.o_pitch, m_obj_desc.o_yaw };

	m_init.coord.clear();
	m_init.coord.resize(2, 0);
	m_init.coord.at(0) = DiscretisationManager::ContXToDiscX(m_init.state.at(0));
	m_init.coord.at(1) = DiscretisationManager::ContYToDiscY(m_init.state.at(1));
	// VisualiseState(m_init, "start_state", 90);

	computeGoal();
	createLatticeAndSearch();
	m_input_push.clear();

	return true;
}

void Agent::ComputeNGRComplement(
	double ox, double oy, double oz,
	double sx, double sy, double sz, bool vis)
{
	int x_c, y_c, z_c;
	m_ngr_grid->worldToGrid(ox + (sx / 2.0), oy + (sy / 2.0), oz + (sz / 2.0), x_c, y_c, z_c);
	double res = m_ngr_grid->resolution();
	int x_s = ((sx / 2.0) / res) + 0.5;
	int y_s = ((sy / 2.0) / res) + 0.5;
	int z_s = (((sz / 2.0) - res) / res) + 0.5;

	std::vector<Eigen::Vector3d> complement_voxel_vecs;
	for (int x = x_c - x_s; x < x_c + x_s; ++x)
	{
		for (int y = y_c - y_s; y < y_c + y_s; ++y)
		{
			bool complement = true;
			double wx, wy, wz;
			for (int z = z_c - z_s; z < z_c + z_s; ++z)
			{
				if (m_ngr_grid->getDistanceField()->getCellDistance(x, y, z) <= 0.0 ||
					m_obs_grid->getDistanceField()->getCellDistance(x, y, z) <= 0.0)
				{
					complement = false;
					break;
				}
			}
			if (complement)
			{
				m_ngr_grid->gridToWorld(x, y, z_c - z_s, wx, wy, wz);
				LatticeState s;
				s.state = {wx, wy};

				if (!stateObsCollision(s))
				{
					if (stateOutsideNGR(s))
					{
						m_ngr_complement.emplace(wx, wy, wz);
						if (vis) {
							complement_voxel_vecs.emplace_back(wx, wy, wz);
						}
					}
				}
			}
		}
	}

	if (vis)
	{
		SV_SHOW_INFO(smpl::visual::MakeCubesMarker(
						complement_voxel_vecs,
						m_ngr_grid->resolution(),
						smpl::visual::Color{ 0.8f, 0.255f, 1.0f, 1.0f },
						m_planning_frame,
						"complement"));
	}
}

void Agent::ResetInvalidPushes(
	const std::vector<std::pair<Coord, Coord> >* invalids_G,
	const std::map<Coord, int, coord_compare>* invalids_L)
{
	m_lattice->ResetInvalidPushes(invalids_G, invalids_L);
}

// const bgi::rtree<value, bgi::quadratic<8> >& Agent::GetInvalidPushes() const
// {
// 	return m_lattice->GetInvalidPushes();
// }

void Agent::AddHallucinatedConstraint(const Coord &c)
{
	m_lattice->AddHallucinatedConstraint(c);
}

int Agent::InvalidPushCount(const Coord &c)
{
	return m_lattice->InvalidPushCount(c);
}

bool Agent::SatisfyPath(
	HighLevelNode* ct_node,
	Trajectory** sol_path,
	int& expands,
	int& min_f,
	std::unordered_set<int>* to_avoid)
{
	m_solve.clear();
	expands = 0;
	min_f = 0;

	// collect agent constraints
	m_lattice->SetCTNode(ct_node);
	if (to_avoid != nullptr) {
		m_lattice->AvoidAgents(*to_avoid);
	}

	std::vector<int> solution;
	int solcost;
	bool result = m_search->replan(&solution, &solcost);

	if (result)
	{
		m_lattice->ConvertPath(solution);
		*sol_path = &(this->m_solve);
		expands = m_search->get_n_expands();
		min_f = m_search->get_min_f();
		// SV_SHOW_INFO_NAMED("movable_trajectory", makePathVisualization());
	}

	return result;
}

bool Agent::InitPP()
{
	m_pp = true;
	m_t = 0;
	m_init.t = 0;
	m_init.hc = 0;
	m_init.state.clear();
	m_init.state = { 	m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z,
						m_obj_desc.o_roll, m_obj_desc.o_pitch, m_obj_desc.o_yaw };

	m_init.coord.clear();
	m_init.coord.resize(2, 0);
	m_init.coord.at(0) = DiscretisationManager::ContXToDiscX(m_init.state.at(0));
	m_init.coord.at(1) = DiscretisationManager::ContYToDiscY(m_init.state.at(1));
}

bool Agent::PlanPrioritised(int p)
{
	m_priority = p;
	m_goal = m_init.coord;

	m_lattice = std::make_unique<AgentLattice>();
	m_lattice->init(this);
	m_lattice->reset();

	m_search = std::make_unique<WAStar>(m_lattice.get(), 1.0);
	m_search->reset();

	m_search->push_start(m_lattice->PushStart(m_init));
	m_search->push_goal(m_lattice->PushGoal(m_goal));

	std::vector<int> solution;
	int solcost;
	bool result = m_search->replan(&solution, &solcost);

	if (result)
	{
		m_lattice->ConvertPath(solution);
		m_cc->UpdateTraj(m_priority, m_solve);
	}

	return result;
}

bool Agent::PrioritisedCollisionCheck(const LatticeState& s, bool goal_check)
{
	if (m_priority == 0) {
		return false; // cannot be in collision
	}
	return m_cc->PPCollisionCheck(this, s, m_obj.GetFCLObject(), m_priority, goal_check);
}

void Agent::UpdatePose(const LatticeState& s)
{
	m_obj.UpdatePose(s);
}

bool Agent::OutOfBounds(const LatticeState& s)
{
	return m_cc->OutOfBounds(s);
}

bool Agent::ImmovableCollision()
{
	return m_cc->ImmovableCollision(m_obj.GetFCLObject());
}

bool Agent::ObjectObjectCollision(const int& a2_id, const LatticeState& a2_q)
{
	return m_cc->ObjectObjectCollision(m_obj.GetFCLObject(), a2_id, a2_q);
}

bool Agent::ObjectObjectsCollision(
	const std::vector<int>& other_ids,
	const std::vector<LatticeState>& other_poses)
{
	return m_cc->ObjectObjectsCollision(m_obj.GetFCLObject(), other_ids, other_poses);
}

double Agent::ObstacleGaussianCost(double x, double y) const
{
	return m_cc->ObstacleGaussianCost(x, y);
}

bool Agent::OutsideNGR(const LatticeState& s)
{
	return stateOutsideNGR(s);
}

double Agent::ObsDist(double x, double y)
{
	LatticeState s;
	s.state = { x, y, m_obj_desc.o_z };
	m_obj.UpdatePose(s);
	double dist = m_cc->ObstacleDist(m_obj.GetFCLObject());
	m_obj.UpdatePose(m_init);

	return dist;
}

void Agent::VisualiseState(const Coord& c, const std::string& ns, int hue)
{
	LatticeState s;
	s.state.clear();
	s.state.resize(2, 0.0);
	s.state.at(0) = DiscretisationManager::DiscXToContX(c.at(0));
	s.state.at(1) = DiscretisationManager::DiscYToContY(c.at(1));
	VisualiseState(s, ns, hue);
}

auto Agent::VisualiseState(const LatticeState& s, const std::string& ns, int hue, bool vis)
	-> std::vector<smpl::visual::Marker>
{
	updateObjectTransform(s);

	std::vector<std::vector<double>> sphere_positions;
	std::vector<double> sphere_radii;
	for (auto& sphere : m_obj.SpheresState()->spheres)
	{
		if (!sphere.isLeaf()) {
			continue;
		}
		m_obj.updateSphereState(smpl::collision::SphereIndex(sphere.parent_state->index, sphere.index()));

		sphere_positions.push_back({ sphere.pos.x(), sphere.pos.y(), sphere.pos.z() });
		sphere_radii.push_back(sphere.model->radius);
	}

	auto ma = viz::getSpheresMarkerArray(
		sphere_positions, sphere_radii, hue, "", "agent_state", GetID());
	for (auto& m : ma.markers) {
		m.header.frame_id = m_ngr_grid->getReferenceFrame();
	}

	std::vector<smpl::visual::Marker> markers;
	markers.reserve(ma.markers.size());
	smpl::visual::Marker m;
	for (auto& mm : ma.markers) {
		smpl::visual::ConvertMarkerMsgToMarker(mm, m);
		markers.push_back(m);
	}
	for (auto& marker : markers) {
		marker.ns = ns;
	}

	if (vis) {
		SV_SHOW_INFO_NAMED(ns, markers);
	}

	return markers;
}

bool Agent::GetSE2Push(std::vector<double>& push, bool input)
{
	if (input)
	{
		if (m_input_push.empty())
		{
			double x0, y0, x1, y1;
			SMPL_WARN("Please specify push start location (x0, y0)");
			std::cin >> x0 >> y0;
			SMPL_WARN("Please specify push end location (x1, y1)");
			std::cin >> x1 >> y1;

			m_input_push.push_back(x0);
			m_input_push.push_back(y0);
			m_input_push.push_back(x1);
			m_input_push.push_back(y1);
		}

		push.clear();
		push.resize(5, 0.0);

		// push start
		push[0] = m_input_push[0];
		push[1] = m_input_push[1];
		// push end
		push[3] = m_input_push[2];
		push[4] = m_input_push[3];

		// push direction
		double move_dir = std::atan2(
					push[4] - push[1],
					push[3] - push[0]);
		push[2] = move_dir;

		return true;
	}

	push.clear();
	push.resize(4, 0.0);
	double move_dir = std::atan2(
					m_solve.back().state.at(1) - m_solve.front().state.at(1),
					m_solve.back().state.at(0) - m_solve.front().state.at(0));

	// default values
	push[0] = m_obj_desc.o_x;
	push[1] = m_obj_desc.o_y;
	push[2] = move_dir;

	return m_obj.GetSE2Push(push, move_dir, m_init);
}

void Agent::GetVoxels(const ContPose& pose, std::set<Coord, coord_compare>& voxels)
{
	m_obj.updateVoxelsState(pose.GetTransform());
	auto voxels_state = m_obj.VoxelsState();

	voxels.clear();
	for (const Eigen::Vector3d& v : voxels_state->voxels)
	{
		Coord c;
		c.push_back(DiscretisationManager::ContXToDiscX(v[0]));
		c.push_back(DiscretisationManager::ContYToDiscY(v[1]));
		c.push_back(DiscretisationManager::ContYToDiscY(v[2]));
		voxels.insert(c);
	}
}

// find best NGR complement cell
// 1. if object is fully outside NGR, this is the initial location
// 2. if object is partially inside NGR, this is the "farthest" object cell
// 3. if object is fully inside NGR, this is the closest cell outside NGR
// (ideally would inflate the NGR by the object and then find such a cell)
bool Agent::computeGoal()
{
	m_goal = m_init.coord;
	return true;

	if (stateOutsideNGR(m_init))
	{
		m_goal = m_init.coord;
		return true;
	}
	else
	{
		// VisualiseState(m_init, "init_state", 20);

		double best_dist = m_ngr_grid->getDistanceFromPoint(m_obj_desc.o_x, m_obj_desc.o_y, m_obj_desc.o_z);
		double ox = m_obj_desc.o_x, oy = m_obj_desc.o_y, nx, ny;

		std::random_device dev;
		std::mt19937 rng(dev());
		std::uniform_real_distribution<> D(0.01, 0.02);

		LatticeState curr = m_init;
		curr.coord.clear();
		curr.coord.resize(2, 0);
		int tries = 0;
		while (tries < 25 && !stateOutsideNGR(curr))
		{
			++tries;
			nx = D(rng) * (D(rng) > 0.015 ? 1 : -1);
			ny = D(rng) * (D(rng) > 0.015 ? 1 : -1);
			double new_dist = m_ngr_grid->getDistanceFromPoint(ox + nx, oy + ny, m_obj_desc.o_z);
			if (new_dist > best_dist)
			{
				ox += nx;
				oy += ny;
				curr.state[0] = ox;
				curr.state[1] = oy;
				best_dist = new_dist;
			}
		}

		curr.coord[0] = DiscretisationManager::ContXToDiscX(curr.state[0]);
		curr.coord[1] = DiscretisationManager::ContYToDiscY(curr.state[1]);

		// VisualiseState(curr, "goal_state", 150);

		m_goal = curr.coord;
		return true;
	}
}

bool Agent::createLatticeAndSearch()
{
	if (m_lattice) {
		m_lattice->reset();
	}
	else
	{
		m_lattice = std::make_unique<AgentLattice>();
		m_lattice->init(this);
	}

	if (m_search) {
		m_search->reset();
	}
	else {
		m_search = std::make_unique<Focal>(m_lattice.get(), 2.0);
	}
	m_search->push_start(m_lattice->PushStart(m_init));
	m_search->push_goal(m_lattice->PushGoal(m_goal));

	return true;
}

// return false => no collision with obstacles
bool Agent::stateObsCollision(const LatticeState& s)
{
	m_obj.UpdatePose(s);
	return m_cc->ImmovableCollision(m_obj.GetFCLObject());
}

bool Agent::updateObjectTransform(const LatticeState& s)
{
	double r = 0.0, p = 0.0, y = 0.0;
	if (s.state.size() == 6)
	{
		r = s.state[3];
		p = s.state[4];
		y = s.state[5];
	}

	Eigen::Affine3d T = Eigen::Translation3d(s.state[0], s.state[1], m_obj_desc.o_z) *
						Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

	m_obj.SetTransform(T);
}

// return false => collide with NGR
bool Agent::stateOutsideNGR(const LatticeState& s, double &dist)
{
	updateObjectTransform(s);
	std::vector<const smpl::collision::CollisionSphereState*> q = {
									m_obj.SpheresState()->spheres.root() };

	dist = -1.0;
	double padding = 0.0;
	return smpl::collision::CheckVoxelsCollisions(
							m_obj, q, *(m_ngr_grid.get()), padding, dist);
}

bool Agent::stateOutsideNGR(const LatticeState& s)
{
	double dist;
	return stateOutsideNGR(s, dist);
}

auto Agent::makePathVisualization()
	-> std::vector<smpl::visual::Marker>
{
	std::vector<smpl::visual::Marker> ma;

	if (m_solve.empty()) {
		return ma;
	}

	for (size_t i = 0; i < m_solve.size(); ++i)
	{
		auto markers = VisualiseState(m_solve.at(i), "movable_trajectory", 180 + i, false);
		for (auto& m : markers) {
			ma.push_back(std::move(m));
		}
	}

	for (size_t i = 0; i < ma.size(); ++i)
	{
		auto& marker = ma[i];
		marker.ns = "movable_trajectory";
		marker.id = i;
	}

	return ma;
}

} // namespace clutter
