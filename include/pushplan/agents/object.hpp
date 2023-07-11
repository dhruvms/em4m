#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <pushplan/utils/types.hpp>

#include <sbpl_collision_checking/base_collision_models.h>
#include <sbpl_collision_checking/base_collision_states.h>
#include <sbpl_collision_checking/shapes.h>
#include <fcl/collision_object.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>

namespace clutter
{

struct ObjectDesc
{
	int id, shape, type;
	double o_x, o_y, o_z;
	double o_roll, o_pitch, o_yaw;
	double x_size, y_size, z_size;
	double mass, mu, yaw_offset;
	bool movable, locked, ycb;
};

struct Object
{
	ObjectDesc desc;

	// CollisionObjects
	moveit_msgs::CollisionObject* moveit_obj = nullptr;
	fcl::CollisionObject* fcl_obj = nullptr;

	// SMPLCollisionObject
	smpl::collision::CollisionShape* smpl_shape = nullptr;
	smpl::collision::CollisionObject* smpl_co = nullptr;

	// CollisionModels
	smpl::collision::CollisionSpheresModel* spheres_model = nullptr;
	smpl::collision::CollisionSpheresState* spheres_state = nullptr;
    smpl::collision::CollisionVoxelsModel* voxels_model = nullptr;
    smpl::collision::CollisionVoxelsState* voxels_state = nullptr;

	int Shape() const;
	double Height() const;
	bool Symmetric() const;
	double GaussianCost(double x, double y) const;
	bool GetSE2Push(std::vector<double>& push, double dir, LatticeState from);

	void SetupGaussianCost();
	bool CreateCollisionObjects();
	bool CreateSMPLCollisionObject();
	bool GenerateCollisionModels();

	void SetTransform(const Eigen::Affine3d& T) { m_T = T; };
	void updateSphereState(const smpl::collision::SphereIndex& sidx);
	void updateVoxelsState(const Eigen::Affine3d& T);

	// bool TransformAndVoxelise(
	// 	const Eigen::Affine3d& transform,
	// 	const double& res, const Eigen::Vector3d& origin,
	// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);
	// bool Voxelise(
	// 	const double& res, const Eigen::Vector3d& origin,
	// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax);

	auto SpheresState() -> smpl::collision::CollisionSpheresState* { return spheres_state; }
	auto VoxelsState() -> smpl::collision::CollisionVoxelsState* { return voxels_state; }

	void UpdatePose(const LatticeState& s);
	fcl::AABB ComputeAABBTight();

	void GetMoveitObj(moveit_msgs::CollisionObject& msg) const {
		msg = *moveit_obj;
	};
	fcl::CollisionObject* GetFCLObject() { return fcl_obj; };

private:
	Eigen::Affine3d m_T;

	double m_log_det;
	Eigen::Matrix2d m_U;

	bool createSpheresModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
	    const smpl::collision::Affine3dVector& transforms);
	bool createVoxelsModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
	    const smpl::collision::Affine3dVector& transforms);
	bool generateSpheresModel(
		const std::vector<shapes::ShapeConstPtr>& shapes,
		const smpl::collision::Affine3dVector& transforms,
		smpl::collision::CollisionSpheresModelConfig& spheres_model);
	void generateVoxelsModel(
		smpl::collision::CollisionVoxelModelConfig& voxels_model);
	bool voxelizeAttachedBody(
		const std::vector<shapes::ShapeConstPtr>& shapes,
		const smpl::collision::Affine3dVector& transforms,
		smpl::collision::CollisionVoxelsModel& model) const;
};

class ContPose;
class DiscPose;
class ObjectState;

class ContPose
{
public:
	ContPose() = default;
	ContPose(const ContPose& other) = default;
	ContPose(double x, double y, double z, double roll, double pitch, double yaw);
	ContPose(const DiscPose &disc_pose);

	const double &x() const {
		return m_x;
	}
	const double &y() const {
		return m_y;
	}
	const double &z() const {
		return m_z;
	}
	const double &roll() const {
		return m_roll;
	}
	const double &pitch() const {
		return m_pitch;
	}
	const double &yaw() const {
		return m_yaw;
	}
	Eigen::Affine3d GetTransform() const;

	bool operator==(const ContPose &other) const;
	bool operator!=(const ContPose &other) const;

private:
	double m_x = 0.0;
	double m_y = 0.0;
	double m_z = 0.0;
	double m_roll = 0.0;
	double m_pitch = 0.0;
	double m_yaw = 0.0;
};

class DiscPose
{
public:
	DiscPose() = default;
	DiscPose(const DiscPose& other) = default;
	DiscPose(int x, int y, int z, int roll, int pitch, int yaw);
	DiscPose(const ContPose &cont_pose);

	const int &x() const {
		return m_x;
	}
	const int &y() const {
		return m_y;
	}
	const int &z() const {
		return m_z;
	}
	const int &roll() const {
		return m_roll;
	}
	const int &pitch() const {
		return m_pitch;
	}
	const int &yaw() const {
		return m_yaw;
	}

	bool operator==(const DiscPose &other) const;
	bool operator!=(const DiscPose &other) const;
	bool EqualsPosition(const DiscPose &other) const;

private:
	int m_x = 0;
	int m_y = 0;
	int m_z = 0;
	int m_roll = 0;
	int m_pitch = 0;
	int m_yaw = 0;
};

class ObjectState
{
public:
	ObjectState();
	ObjectState(int id, bool symmetric, const ContPose &cont_pose);
	ObjectState(int id, bool symmetric, const DiscPose &disc_pose);

	const int &id() const {
		return m_id;
	}
	const bool &symmetric() const {
		return m_symmetric;
	}
	const ContPose &cont_pose() const {
		return m_cont_pose;
	}
	const DiscPose &disc_pose() const {
		return m_disc_pose;
	}

	// Two object states are equal if they have the same ID and have the same discrete pose (up to symmetry).
	bool operator==(const ObjectState &other) const;
	bool operator!=(const ObjectState &other) const;

private:
	int m_id;
	bool m_symmetric;
	ContPose m_cont_pose;
	DiscPose m_disc_pose;
};

std::ostream &operator<< (std::ostream &stream, const DiscPose &disc_pose);
std::ostream &operator<< (std::ostream &stream, const ContPose &cont_pose);
std::ostream &operator<< (std::ostream &stream,
							const ObjectState &object_state);

} // namespace clutter


#endif // OBJECT_HPP
