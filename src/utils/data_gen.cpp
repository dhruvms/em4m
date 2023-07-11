#include <pushplan/utils/data_gen.hpp>
#include <pushplan/utils/helpers.hpp>

#include <string>

namespace clutter
{

DataGeneration::DataGeneration() : m_rng(m_dev()), m_ph("~")
{
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
}

void DataGeneration::Reset()
{
	if (m_robot)
	{
		m_sim.reset();
		m_robot.reset();
	}

	std::string tables;
	int num_immov, num_mov;

	m_ph.getParam("object_filename", tables);
	m_ph.getParam("objects/num_immov", num_immov);
	m_ph.getParam("objects/num_mov", num_mov);

	m_sim = std::make_unique<BulletSim>(
				tables, false,
				-1, std::string(),
				0, 1, true); // z_offset = true randomly displaces table/shelf along z-axis

	m_robot = std::make_unique<Robot>();
	m_robot->SetSim(m_sim.get());
	m_robot->Setup();
	m_robot->AddObstaclesFromSim();

	setupSim(m_sim.get(), m_robot->GetStartState()->joint_state, -1);
}

void DataGeneration::GetPushData()
{
	auto mov_objs = m_sim->GetMovableObjs();
	auto mov_obj_ids = m_sim->GetMovableObjIDs();
	Object movable;
	movable.desc.id = mov_obj_ids->front().first;
	movable.desc.shape = mov_obj_ids->front().second;
	movable.desc.type = 1; // movable
	movable.desc.o_x = mov_objs->front().at(0);
	movable.desc.o_y = mov_objs->front().at(1);
	movable.desc.o_z = mov_objs->front().at(2);
	movable.desc.o_roll = mov_objs->front().at(3);
	movable.desc.o_pitch = mov_objs->front().at(4);
	movable.desc.o_yaw = mov_objs->front().at(5);
	movable.desc.ycb = (bool)mov_objs->front().back();

	if (movable.desc.ycb)
	{
		auto itr = YCB_OBJECT_DIMS.find(mov_obj_ids->front().second);
		if (itr != YCB_OBJECT_DIMS.end())
		{
			movable.desc.x_size = itr->second.at(0);
			movable.desc.y_size = itr->second.at(1);
			movable.desc.z_size = itr->second.at(2);
			movable.desc.o_yaw += itr->second.at(3);
		}
		movable.desc.mass = -1;
		movable.desc.mu = mov_objs->front().at(6);
	}
	else
	{
		movable.desc.x_size = mov_objs->front().at(6);
		movable.desc.y_size = mov_objs->front().at(7);
		movable.desc.z_size = mov_objs->front().at(8);
		movable.desc.mass = mov_objs->front().at(9);
		movable.desc.mu = mov_objs->front().at(10);
	}
	movable.desc.movable = true;
	movable.desc.locked = false;

	movable.CreateCollisionObjects();
	movable.CreateSMPLCollisionObject();
	movable.GenerateCollisionModels();

	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");

	filename = filename.substr(0, found + 1) + "../../dat/push_data/PUSH_DATA";
	int modifier;
	m_ph.getParam("robot/study", modifier);
	filename += "_" + std::to_string(modifier) + ".csv";

	bool exists = FileExists(filename);
	std::ofstream DATA;
	DATA.open(filename, std::ofstream::out | std::ofstream::app);
	if (!exists)
	{
		DATA 	<< "o_ox,o_oy,o_oz,o_oyaw,o_shape,o_xs,o_ys,o_zs,o_mass,o_mu,"
				<< "m_dir,m_dist,"
				<< "p_x,p_y,p_z,"
				<< "p_r11,p_r21,p_r31,p_r12,p_r22,p_r32,p_r13,p_r23,p_r33,"
				<< "r\n";
	}

	for (int i = 0; i < 25; ++i)
	{
		std::vector<double> push;
		double move_dir, move_dist;
		Eigen::Affine3d start_pose;
		int result;
		m_robot->GenMovablePush(
			movable,
			push, move_dir, move_dist, start_pose,
			result);
		ROS_WARN("Push sample result: %d", result);

		DATA 	<< movable.desc.o_x << ','
				<< movable.desc.o_y << ','
				<< movable.desc.o_z << ','
				<< movable.desc.o_yaw << ','
				<< movable.desc.shape << ','
				<< movable.desc.x_size << ','
				<< movable.desc.y_size << ','
				<< movable.desc.z_size << ','
				<< movable.desc.mass << ','
				<< movable.desc.mu << ','
				<< move_dir << ','
				<< move_dist << ','
				<< start_pose.translation().x() << ','
				<< start_pose.translation().y() << ','
				<< start_pose.translation().z() << ','
				<< start_pose.rotation()(0, 0) << ','
				<< start_pose.rotation()(1, 0) << ','
				<< start_pose.rotation()(2, 0) << ','
				<< start_pose.rotation()(0, 1) << ','
				<< start_pose.rotation()(1, 1) << ','
				<< start_pose.rotation()(2, 1) << ','
				<< start_pose.rotation()(0, 2) << ','
				<< start_pose.rotation()(1, 2) << ','
				<< start_pose.rotation()(2, 2) << ','
				<< result << '\n';
	}
	DATA.close();
}

} // namespace clutter
