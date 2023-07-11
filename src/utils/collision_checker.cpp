#include <pushplan/utils/collision_checker.hpp>
#include <pushplan/agents/robot.hpp>
#include <pushplan/search/planner.hpp>
#include <pushplan/utils/geometry.hpp>
#include <pushplan/utils/default_broadphase_callbacks.h>

#include <smpl/console/console.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace clutter
{

CollisionChecker::CollisionChecker(Planner* planner, const std::vector<Object>& obstacles, std::shared_ptr<Robot> r)
:
m_planner(planner),
m_obstacles(obstacles),
m_rng(m_dev())
{
	m_fcl_immov = new fcl::DynamicAABBTreeCollisionManager();
	// preprocess immovable obstacles
	for (size_t i = 0; i != m_obstacles.size(); ++i)
	{
		if (m_obstacles.at(i).desc.id == 1) {
			m_base_loc = i;
			continue;
		}
		LatticeState s;
		s.state.push_back(m_obstacles.at(i).desc.o_x);
		s.state.push_back(m_obstacles.at(i).desc.o_y);
		m_obstacles.at(i).UpdatePose(s);
		m_fcl_immov->registerObject(m_obstacles.at(i).GetFCLObject());
	}
	m_fcl_immov->setup();

	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
	m_robot = r;
}

void CollisionChecker::ReinitMovableCC(const std::vector<std::shared_ptr<Agent> >& movables)
{
	if (m_fcl_mov != nullptr) {
		m_fcl_mov->clear();
	}

	m_fcl_mov = new fcl::DynamicAABBTreeCollisionManager();

	for(int i = 0; i < movables.size(); i++) {
		m_fcl_mov->registerObject(movables[i]->GetFCLObject());
	}
}

void CollisionChecker::UpdateTraj(const int& priority, const Trajectory& traj)
{
	if (int(m_trajs.size()) <= priority) {
		m_trajs.resize(priority + 1, {});
	}

	m_trajs.at(priority) = traj;
}

bool CollisionChecker::OutOfBounds(const LatticeState& s)
{
	bool oob = s.state.at(0) <= (m_obstacles.at(m_base_loc).desc.o_x - m_obstacles.at(m_base_loc).desc.x_size);
	oob = oob || s.state.at(0) >= (m_obstacles.at(m_base_loc).desc.o_x + m_obstacles.at(m_base_loc).desc.x_size);
	oob = oob || s.state.at(1) <= (m_obstacles.at(m_base_loc).desc.o_y - m_obstacles.at(m_base_loc).desc.y_size);
	oob = oob || s.state.at(1) >= (m_obstacles.at(m_base_loc).desc.o_y + m_obstacles.at(m_base_loc).desc.y_size);

	return oob;
}

bool CollisionChecker::ImmovableCollision(const State& s, fcl::CollisionObject* o)
{
	LatticeState ls;
	ls.state = s;

	fcl::Vec3f t(o->getTranslation());
	t.setValue(ls.state[0], ls.state[1], t[2]);
	Eigen::Quaterniond q_eigen;
	smpl::angles::from_euler_zyx(ls.state[5], ls.state[4], ls.state[3], q_eigen);
	fcl::Quaternion3f q_fcl(q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
	fcl::Transform3f pose;
	pose.setTranslation(t);
	pose.setQuatRotation(q_fcl);

	o->setTransform(pose);
	o->computeAABB();

	return this->ImmovableCollision(o);
}

bool CollisionChecker::ImmovableCollision(fcl::CollisionObject* o)
{
	// double start_time = GetTime(), time_taken;

	fcl::DefaultCollisionData collision_data;
	m_fcl_immov->collide(o, &collision_data, fcl::DefaultCollisionFunction);

	// time_taken = GetTime() - start_time;
	// SMPL_INFO("Immovable collision check: %f seconds.", time_taken);

	return collision_data.result.isCollision();
}

// called by CBS::findConflicts
bool CollisionChecker::ObjectObjectCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
{
	fcl::CollisionRequest request;
	fcl::CollisionResult result;
	fcl::collide(o1, o2, request, result);
	return result.isCollision();
}

// called by Agent::generateSuccessor
bool CollisionChecker::ObjectObjectCollision(fcl::CollisionObject* o1, const int& a2_id, const LatticeState& a2_q)
{
	auto a2 = m_planner->GetAgent(a2_id);
	a2->UpdatePose(a2_q);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;
	fcl::collide(o1, a2->GetFCLObject(), request, result);
	return result.isCollision();
}

bool CollisionChecker::ObjectObjectsCollision(
	fcl::CollisionObject* o1,
	const std::vector<int>& other_ids,
	const std::vector<LatticeState>& other_poses)
{
	m_fcl_mov->unregisterObject(o1);

	for(int i = 0; i < other_ids.size(); i++)
	{
		auto agent = m_planner->GetAgent(other_ids[i]);
		agent->UpdatePose(other_poses[i]);
		m_fcl_mov->update(agent->GetFCLObject());
	}

	m_fcl_mov->setup();
	fcl::DefaultCollisionData collision_data;
	m_fcl_mov->collide(o1, &collision_data, fcl::DefaultCollisionFunction);

	m_fcl_mov->registerObject(o1);

	return collision_data.result.isCollision();

}

double CollisionChecker::ObstacleGaussianCost(double x, double y) const
{
	double cost = 0.0;

	int tables = FRIDGE ? 5 : 1;
	for (size_t i = tables; i != m_obstacles.size(); ++i)
	{
		if (i == m_base_loc) {
			continue;
		}
		cost += m_obstacles.at(i).GaussianCost(x, y);
	}

	return cost;
}

bool CollisionChecker::PPCollisionCheck(
	Agent* a, const LatticeState& s, fcl::CollisionObject* o, const int& priority, bool goal_check)
{
	m_fcl_mov->unregisterObject(o);
	a->UpdatePose(s);
	auto o_new = a->GetFCLObject();

	// Check against movables' FCL manager
	for (int p = 0; p < priority; ++p)
	{
		if (goal_check)
		{
			auto o2 = m_planner->GetUpdatedObjectFromPriority(m_trajs.at(p).back(), p);
			m_fcl_mov->update(o2);
		}

		else
		{
			if (s.t > m_trajs.at(p).back().t)
			{
				auto o2 = m_planner->GetUpdatedObjectFromPriority(m_trajs.at(p).back(), p);
				m_fcl_mov->update(o2);
				continue;
			}

			for (const auto& s2: m_trajs.at(p))
			{
				if (s.t == s2.t)
				{
					auto o2 = m_planner->GetUpdatedObjectFromPriority(s2, p);
					m_fcl_mov->update(o2);
				}
			}
		}
	}
	m_fcl_mov->setup();
	fcl::DefaultCollisionData collision_data;
	m_fcl_mov->collide(o_new, &collision_data, fcl::DefaultCollisionFunction);
	bool collision = collision_data.result.isCollision();

	m_fcl_mov->registerObject(o_new);
	return collision;
}

double CollisionChecker::ObstacleDist(fcl::CollisionObject* o)
{
	fcl::DefaultDistanceData distance_data;
	m_fcl_immov->distance(o, &distance_data, fcl::DefaultDistanceFunction);
	return distance_data.result.min_distance;
}

bool CollisionChecker::RobotObjectCollision(
	Agent* a1, const LatticeState& a1_state,
	const LatticeState& robot_state,
	int t, bool process)
{
	SMPL_ERROR("CollisionChecker::RobotObjectCollision un-implemented!");
	return false;
}

State CollisionChecker::GetRandomStateOutside(fcl::CollisionObject* o)
{
	State g(2, 0.0);
	State gmin(2, 0.0), gmax(2, 0.0);

	gmin.at(0) = OutsideXMin();
	gmax.at(0) = OutsideXMax();

	gmin.at(1) = OutsideYMin();
	gmax.at(1) = OutsideYMax();

	do
	{
		g.at(0) = (m_distD(m_rng) * (gmax.at(0) - gmin.at(0))) + gmin.at(0);
		g.at(1) = (m_distD(m_rng) * (gmax.at(1) - gmin.at(1))) + gmin.at(1);
	}
	while (ImmovableCollision(g, o));

	return g;
}

bool CollisionChecker::checkCollisionObjSet(
	const Object& o1, const State& o1_loc,
	bool rect_o1, const std::vector<State>& o1_rect,
	const std::vector<Object>* a2_objs)
{
	State o2_loc;
	bool rect_o2;
	std::vector<State> o2_rect;

	for (const auto& ao: *a2_objs)
	{
		rect_o2 = false;
		o2_loc = {ao.desc.o_x, ao.desc.o_y};
		if (ao.Shape() == 0)
		{
			GetRectObjAtPt(o2_loc, ao.desc, o2_rect);
			rect_o2 = true;
		}

		if (rect_o1)
		{
			if (rect_o2)
			{
				if (rectRectCollision(o1_rect, o2_rect)) {
					return false;
				}
			}
			else
			{
				if (rectCircCollision(o1_rect, ao, o2_loc)) {
					return false;
				}
			}
		}
		else
		{
			if (rect_o2)
			{
				if (rectCircCollision(o2_rect, o1, o1_loc)) {
					return false;
				}
			}
			else
			{
				if (circCircCollision(o1, o1_loc, ao, o2_loc)) {
					return false;
				}
			}
		}
	}

	return true;
}

bool CollisionChecker::rectRectCollision(
	const std::vector<State>& r1, const std::vector<State>& r2)
{
	return RectanglesIntersect(r1, r2);
}

bool CollisionChecker::rectCircCollision(
	const std::vector<State>& r1, const Object& c1, const State& c1_loc)
{
	return (PointInRectangle(c1_loc, r1) ||
			LineSegCircleIntersect(c1_loc, c1.desc.x_size, r1.at(0), r1.at(1)) ||
			LineSegCircleIntersect(c1_loc, c1.desc.x_size, r1.at(1), r1.at(2)) ||
			LineSegCircleIntersect(c1_loc, c1.desc.x_size, r1.at(2), r1.at(3)) ||
			LineSegCircleIntersect(c1_loc, c1.desc.x_size, r1.at(3), r1.at(0)));
}

bool CollisionChecker::circCircCollision(
	const Object& c1, const State& c1_loc,
	const Object& c2, const State& c2_loc)
{
	double dist = EuclideanDist(c1_loc, c2_loc);
	return (dist < (c1.desc.x_size + c2.desc.x_size));
}

} // namespace clutter
