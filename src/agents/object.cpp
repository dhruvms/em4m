#include <pushplan/agents/object.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/discretisation.hpp>
#include <pushplan/utils/helpers.hpp>

#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/voxel_operations.h>
#include <smpl/angles.h>
#include <smpl/stl/memory.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

namespace clutter
{

int Object::Shape() const
{
	if (!desc.ycb) {
		return desc.shape;
	}
	else {
		if (desc.shape == 36) {
			return 0;
		}
		int s = desc.x_size == desc.y_size ? 2 : 0;
		return s;
	}
}

double Object::Height() const
{
	if (this->Shape() == 0) {
		return desc.z_size * 2;
	}
	return desc.z_size;
}

bool Object::Symmetric() const
{
	if (!desc.ycb)
	{
		if (desc.shape == 0) {
			return std::abs(desc.x_size - desc.y_size) <= RES; // cuboids are symmetric up to resolution
		}
		return true; // cylinders are symmetric
	}
	else
	{
		if (desc.shape == 2 || desc.shape == 5 || desc.shape == 19 ||
				desc.shape == 25 || desc.shape == 36) {
			return true;
		}
		else {
			return false;
		}
	}
}

double Object::GaussianCost(double x, double y) const
{
	Eigen::Vector2d dev;
	dev << x - desc.o_x, y - desc.o_y;

	Eigen::Vector2d mahalanobis = dev.transpose() * m_U;
	mahalanobis = mahalanobis.array().square();

	// assuming Gaussian covariance is always full rank (= 2)
	double logval = -0.5 * (2 * LOG2PI + m_log_det + mahalanobis.sum());
	double expval = std::exp(logval);
	return expval * 1e6;
}

bool Object::GetSE2Push(std::vector<double>& push, double dir, LatticeState from)
{
	if (!from.state.empty()) {
		this->UpdatePose(from);
	}

	// Ray-AABB intersection code from
	// https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/ray-tracing-rendering-simple-shapes&src=1

	// AABB bounds
	auto aabb = this->ComputeAABBTight();
	std::vector<fcl::Vec3f> bounds = {aabb.min_, aabb.max_};

	// Push direction and inverse direction
	Eigen::Vector3f push_dir(
			std::cos(dir + M_PI),
			std::sin(dir + M_PI),
			0.0);
	push_dir.normalize();
	Eigen::Vector3f inv_dir = push_dir.array().inverse();

	// Push direction sign vector
	Eigen::Vector3i push_sign(
		inv_dir[0] < 0, inv_dir[1] < 0, inv_dir[2] < 0);

	float tmin, tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[push_sign[0]][0] - desc.o_x) * inv_dir[0];
	tmax = (bounds[1 - push_sign[0]][0] - desc.o_x) * inv_dir[0];
	tymin = (bounds[push_sign[1]][1] - desc.o_y) * inv_dir[1];
	tymax = (bounds[1 - push_sign[1]][1] - desc.o_y) * inv_dir[1];

	if ((tmin > tymax) || (tymin > tmax)) {
		return false;
	}

	if (tymin > tmin) {
		tmin = tymin;
	}
	if (tymax < tmax) {
		tmax = tymax;
	}

	tzmin = (bounds[push_sign[2]][2] - desc.o_z) * inv_dir[2];
	tzmax = (bounds[1 - push_sign[2]][2] - desc.o_z) * inv_dir[2];

	if ((tmin > tzmax) || (tzmin > tmax)) {
		return false;
	}

	if (tzmin > tmin) {
		tmin = tzmin;
	}
	if (tzmax < tmax) {
		tmax = tzmax;
	}

	float t = tmin;

	if (t < 0)
	{
		t = tmax;
		if (t < 0) {
			return false;
		}
	}

	if (t > 1) {
		return false;
	}

	push[0] = desc.o_x + push_dir[0] * t;
	push[1] = desc.o_y + push_dir[1] * t;
	push[3] = t;
	return true;
}

void Object::SetupGaussianCost()
{
	double sx2, sy2, cth, sth, cth2, sth2, cov;
	sx2 = std::pow(desc.x_size, 2);
	sy2 = std::pow(desc.y_size, 2);
	cth = std::cos(desc.o_yaw);
	sth = std::sin(desc.o_yaw);
	cth2 = std::pow(cth, 2);
	sth2 = std::pow(sth, 2);
	cov = (sx2 - sy2)*cth*sth;

	Eigen::Matrix2d Sigma, D, V, D_pinv_sqrt;
	Sigma <<	sx2*cth2 + sy2*sth2, cov,
				cov, sx2*sth2 + sy2*cth2;
	Sigma /= 1.386;
	Eigen::EigenSolver<Eigen::Matrix2d> eig_d(Sigma);

	D = eig_d.pseudoEigenvalueMatrix();
	V = eig_d.pseudoEigenvectors();
	D_pinv_sqrt = D.inverse().array().sqrt();

	m_U = V * D_pinv_sqrt;
	m_log_det = D.diagonal().array().log().sum();
}

bool Object::CreateCollisionObjects()
{
	moveit_obj = new moveit_msgs::CollisionObject();
	moveit_obj->id = std::to_string(desc.id);

	geometry_msgs::Pose pose;
	Eigen::Quaterniond q;
	geometry_msgs::Quaternion orientation;
	if (!desc.ycb)
	{
		shape_msgs::SolidPrimitive prim;

		if (desc.shape == 0)
		{
			// Create Moveit collision object shape
			prim.type = shape_msgs::SolidPrimitive::BOX;
			prim.dimensions.resize(3);
			prim.dimensions[0] = desc.x_size * 2;
			prim.dimensions[1] = desc.y_size * 2;
			prim.dimensions[2] = desc.z_size * 2;

			// Create FCL collision object
			std::shared_ptr<fcl::Box> fcl_shape =
				std::make_shared<fcl::Box>(desc.x_size * 2, desc.y_size * 2, desc.z_size * 2);
			fcl_obj = new fcl::CollisionObject(fcl_shape);
		}
		else if (desc.shape == 2)
		{
			// Create Moveit collision object shape
			prim.type = shape_msgs::SolidPrimitive::CYLINDER;
			prim.dimensions.resize(2);
			prim.dimensions[0] = desc.z_size;
			prim.dimensions[1] = desc.x_size;

			// Create FCL collision object
			std::shared_ptr<fcl::Cylinder> fcl_shape =
				std::make_shared<fcl::Cylinder>(desc.x_size, desc.z_size);
			fcl_obj = new fcl::CollisionObject(fcl_shape);
		}

		int table_ids = FRIDGE ? 5 : 1;
		if (desc.id <= table_ids)
		{
			prim.dimensions[0] *= (1.0 + (sqrt(3) * DF_RES));
			prim.dimensions[1] *= (1.0 + (sqrt(3) * DF_RES));
			prim.dimensions[2] *= (1.0 + (sqrt(3) * DF_RES));
		}

		pose.position.x = desc.o_x;
		pose.position.y = desc.o_y;
		pose.position.z = desc.o_z;

		smpl::angles::from_euler_zyx(desc.o_yaw, desc.o_pitch, desc.o_roll, q);
		tf::quaternionEigenToMsg(q, orientation);

		pose.orientation = orientation;
		moveit_obj->primitives.push_back(prim);
		moveit_obj->primitive_poses.push_back(pose);
	}
	else
	{
		// Create Moveit collision object
		std::string stl_mesh(__FILE__);
		auto found = stl_mesh.find_last_of("/\\");
		stl_mesh = stl_mesh.substr(0, found + 1) + "../../dat/ycb/?/tsdf/nontextured.stl";

		auto itr1 = YCB_OBJECT_NAMES.find(desc.shape);
		std::string object_name(itr1->second);
		found = stl_mesh.find_last_of("?");
		stl_mesh.insert(found, object_name);
		found = stl_mesh.find_last_of("?");
		stl_mesh.erase(found, 1);
		stl_mesh.insert(0, "file://");

		shapes::Mesh* mesh = nullptr;
		mesh = shapes::createMeshFromResource(stl_mesh, Eigen::Vector3d::Constant(1.0));
		shape_msgs::Mesh mesh_msg;
		shapes::ShapeMsg shape_msg = mesh_msg;
		if (shapes::constructMsgFromShape(mesh, shape_msg)) {
			mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
		}
		else {
			SMPL_ERROR("Failed to get Mesh msg!");
			return false;
		}

		pose.position.x = desc.o_x;
		pose.position.y = desc.o_y;
		pose.position.z = desc.o_z;

		desc.yaw_offset = 0.0;
		auto itr2 = YCB_OBJECT_DIMS.find(desc.shape);
		if (itr2 != YCB_OBJECT_DIMS.end()) {
			desc.yaw_offset = itr2->second.at(3);
		}

		smpl::angles::from_euler_zyx(
				desc.o_yaw - desc.yaw_offset, desc.o_pitch, desc.o_roll, q);
		tf::quaternionEigenToMsg(q, orientation);
		pose.orientation = orientation;

		moveit_obj->meshes.push_back(mesh_msg);
		moveit_obj->mesh_poses.push_back(pose);

		// Create FCL collision object
		auto object_mesh = moveit_obj->meshes[0];

		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;

		vertices.reserve(object_mesh.vertices.size());
		for (unsigned int i = 0; i < object_mesh.vertices.size(); ++i) {
			double x = object_mesh.vertices[i].x;
			double y = object_mesh.vertices[i].y;
			double z = object_mesh.vertices[i].z;
			vertices.push_back(fcl::Vec3f(x, y, z));
		}

		triangles.reserve(object_mesh.triangles.size());
		for (unsigned int i = 0; i < object_mesh.triangles.size(); ++i) {
			fcl::Triangle t;
			t[0] = object_mesh.triangles[i].vertex_indices[0];
			t[1] = object_mesh.triangles[i].vertex_indices[1];
			t[2] = object_mesh.triangles[i].vertex_indices[2];
			triangles.push_back(t);
		}
		typedef fcl::BVHModel<fcl::OBBRSS> Model;
		std::shared_ptr<Model> mesh_geom = std::make_shared<Model>();

		mesh_geom->beginModel();
		mesh_geom->addSubModel(vertices, triangles);
		mesh_geom->endModel();

		fcl_obj = new fcl::CollisionObject(mesh_geom);
	}

	return true;
}

bool Object::CreateSMPLCollisionObject()
{
	std::vector<smpl::collision::CollisionShape*> shapes;
	smpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;

	for (size_t i = 0; i < moveit_obj->primitives.size(); ++i) {
		auto& prim = moveit_obj->primitives[i];

		smpl::collision::CollisionShape* shape;
		switch (prim.type) {
		case shape_msgs::SolidPrimitive::BOX:
			shape = new smpl::collision::BoxShape(
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_X],
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
					prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
			break;
		case shape_msgs::SolidPrimitive::SPHERE:
			shape = new smpl::collision::SphereShape(
					prim.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]);
			break;
		case shape_msgs::SolidPrimitive::CYLINDER:
			shape = new smpl::collision::CylinderShape(
					prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
					prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
			break;
		case shape_msgs::SolidPrimitive::CONE:
			shape = new smpl::collision::ConeShape(
					prim.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
					prim.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]);
			break;
		default:
			return false;
		}

		smpl_shape = shape;
		shapes.push_back(smpl_shape);

		// auto& prim_pose = moveit_obj->primitive_poses[i];
		Eigen::Affine3d transform = Eigen::Translation3d(0.0, 0.0, 0.0) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());;
		// tf::poseMsgToEigen(prim_pose, transform);
		shape_poses.push_back(transform);
	}

	// for (size_t i = 0; i < moveit_obj->planes.size(); ++i) {
	// 	auto& plane = moveit_obj->planes[i];

	// 	auto shape = smpl::make_unique<smpl::collision::PlaneShape>(
	// 			plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]);
	// 	smpl_shape = shape.get();
	// 	shapes.push_back(smpl_shape);

	// 	auto& plane_pose = moveit_obj->plane_poses[i];
	// 	Eigen::Affine3d transform;
	// 	tf::poseMsgToEigen(plane_pose, transform);
	// 	shape_poses.push_back(transform);
	// }

	// for (size_t i = 0; i < moveit_obj->meshes.size(); ++i) {
	// 	auto& mesh = moveit_obj->meshes[i];

	// 	assert(0); // TODO: implement

	// 	auto& mesh_pose = moveit_obj->mesh_poses[i];
	// 	Eigen::Affine3d transform;
	// 	tf::poseMsgToEigen(mesh_pose, transform);
	// 	shape_poses.push_back(transform);
	// }

	// create the collision obj_msg
	smpl_co = new smpl::collision::CollisionObject();
	smpl_co->id = moveit_obj->id;
	smpl_co->shapes = std::move(shapes);
	smpl_co->shape_poses = std::move(shape_poses);

	return true;
}

bool Object::GenerateCollisionModels()
{
	std::vector<shapes::ShapeConstPtr> shapes;
	smpl::collision::Affine3dVector transforms;
	if (smpl_co)
	{
		for (size_t sidx = 0; sidx < smpl_co->shapes.size(); ++sidx)
		{
			auto transform = smpl_co->shape_poses.at(sidx); // Eigen::Affine3d::Identity();
			switch (smpl_co->shapes.at(sidx)->type)
			{
				case smpl::collision::ShapeType::Box:
				{
					auto box = static_cast<smpl::collision::BoxShape*>(smpl_co->shapes.at(sidx));
					shapes::ShapeConstPtr ao_shape(new shapes::Box(box->size[0], box->size[1], box->size[2]));
					shapes.push_back(std::move(ao_shape));
					break;
				}
				case smpl::collision::ShapeType::Cylinder:
				{
					auto cylinder = static_cast<smpl::collision::CylinderShape*>(smpl_co->shapes.at(sidx));
					shapes::ShapeConstPtr ao_shape(new shapes::Cylinder(cylinder->radius, cylinder->height));
					shapes.push_back(std::move(ao_shape));
					break;
				}
				case smpl::collision::ShapeType::Mesh:
				{
					shapes::ShapeConstPtr ao_shape = MakeROSShape(smpl_co->shapes.at(sidx));
					shapes.push_back(std::move(ao_shape));
					break;
				}
				case smpl::collision::ShapeType::Sphere:
				case smpl::collision::ShapeType::Cone:
				case smpl::collision::ShapeType::Plane:
				case smpl::collision::ShapeType::OcTree:
				default:
				{
					ROS_ERROR("Incompatible shape type!");
					return false;
				}
			}
			transforms.push_back(transform);
		}
	}
	else
	{
		ROS_ERROR("Collision object not found!");
		return false;
	}

	if (!createSpheresModel(shapes, transforms)) {
		return false;
	}
	if (!createVoxelsModel(shapes, transforms)) {
		return false;
	}

	return true;
}

// bool Object::TransformAndVoxelise(
// 	const Eigen::Affine3d& transform,
// 	const double& res, const Eigen::Vector3d& origin,
// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax)
// {
// 	// geometry_msgs::Pose pose;
// 	// Eigen::Quaterniond q;
// 	// if (!desc.ycb)
// 	// {
// 	// 	pose.position.x = transform.translation().x();
// 	// 	pose.position.y = transform.translation().y();
// 	// 	pose.position.z = transform.translation().z();

// 	// 	q = Eigen::Quaterniond(transform.rotation());
// 	// 	tf::quaternionEigenToMsg(q, orientation);

// 	// 	pose.orientation = orientation;
// 	// 	moveit_obj->primitive_poses.at(0) = pose;
// 	// }
// 	// else
// 	// {
// 	// 	pose.position.x = transform.translation().x();
// 	// 	pose.position.y = transform.translation().y();
// 	// 	pose.position.z = transform.translation().z();

// 	// 	double yaw_offset = 0.0;
// 	// 	auto itr2 = YCB_OBJECT_DIMS.find(desc.shape);
// 	// 	if (itr2 != YCB_OBJECT_DIMS.end()) {
// 	// 		yaw_offset = itr2->second.at(3);
// 	// 	}

// 	// 	double roll, pitch, yaw;
// 	// 	smpl::angles::get_euler_zyx(transform.rotation(), yaw, pitch, roll);
// 	// 	smpl::angles::from_euler_zyx(
// 	// 			yaw - yaw_offset, pitch, roll, q);
// 	// 	tf::quaternionEigenToMsg(q, orientation);
// 	// 	pose.orientation = orientation;

// 	// 	moveit_obj->mesh_poses.at(0) = pose;
// 	// }

// 	smpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;
// 	// auto& prim_pose = moveit_obj->primitive_poses[i];
// 	// tf::poseMsgToEigen(prim_pose, transform);
// 	shape_poses.push_back(transform);
// 	smpl_co->shape_poses = std::move(shape_poses);

// 	return Voxelise(res, origin, gmin, gmax);
// }

// bool Object::Voxelise(
// 	const double& res, const Eigen::Vector3d& origin,
// 	const Eigen::Vector3d& gmin, const Eigen::Vector3d& gmax)
// {
// 	if (!smpl::collision::VoxelizeObject(*smpl_co, res, origin, gmin, gmax, obj_voxels)) {
// 		return false;
// 	}

// 	return true;
// }


void Object::UpdatePose(const LatticeState& s)
{
	// Update Moveit pose
	geometry_msgs::Pose moveit_pose;
	Eigen::Quaterniond q;
	geometry_msgs::Quaternion orientation;

	moveit_pose.position.x = s.state.at(0);
	moveit_pose.position.y = s.state.at(1);
	moveit_pose.position.z = desc.o_z;

	smpl::angles::from_euler_zyx(desc.o_yaw - desc.yaw_offset, desc.o_pitch, desc.o_roll, q);
	tf::quaternionEigenToMsg(q, orientation);

	moveit_pose.orientation = orientation;

	if (desc.ycb) {
		moveit_obj->mesh_poses[0] = moveit_pose;
	}
	else {
		moveit_obj->primitive_poses[0] = moveit_pose;
	}

	// Update FCL pose
	fcl::Quaternion3f fcl_q(q.w(), q.x(), q.y(), q.z());
	fcl::Vec3f fcl_T(s.state.at(0), s.state.at(1), desc.o_z);
	fcl::Transform3f fcl_pose(fcl_q, fcl_T);

	fcl_obj->setTransform(fcl_pose);
	fcl_obj->computeAABB();
}

fcl::AABB Object::ComputeAABBTight()
{
	auto aabb = fcl_obj->getAABB();
	auto cgeom = fcl_obj->collisionGeometry();
	auto t = fcl_obj->getTransform();
	if (t.getQuatRotation().isIdentity()) {
		aabb = fcl::translate(cgeom->aabb_local, t.getTranslation());
	}
	else
	{
		fcl::Vec3f trans = t.getTranslation();
		fcl::Matrix3f rot = t.getRotation();

		// For all three axes
		for (int i = 0; i < 3; i++)
		{
			// Start by adding in translation
			aabb.min_[i] = aabb.max_[i] = trans[i];

			// Form extent by summing smaller and larger terms respectively
			for (int j = 0; j < 3; j++)
			{
				fcl::FCL_REAL e = rot(i, j) * cgeom->aabb_local.min_[j];
				fcl::FCL_REAL f = rot(i, j) * cgeom->aabb_local.max_[j];

				aabb.min_[i] += (e < f) ? e : f;
				aabb.max_[i] += (e < f) ? f : e;
			}
		}
	}

	return aabb;
}

void Object::updateSphereState(const smpl::collision::SphereIndex& sidx)
{
	smpl::collision::CollisionSphereState& sphere_state = spheres_state->spheres[sidx.s];
	sphere_state.pos = m_T * sphere_state.model->center;
}

void Object::updateVoxelsState(const Eigen::Affine3d& T)
{
	// transform voxels into the model frame
	std::vector<Eigen::Vector3d> new_voxels(voxels_state->model->voxels.size());
	for (size_t i = 0; i < voxels_state->model->voxels.size(); ++i) {
		new_voxels[i] = T * voxels_state->model->voxels[i];
	}

	voxels_state->voxels = std::move(new_voxels);
}

bool Object::createSpheresModel(
	const std::vector<shapes::ShapeConstPtr>& shapes,
	const smpl::collision::Affine3dVector& transforms)
{
	ROS_DEBUG("	Generate spheres model");

	// create configuration spheres and a spheres model for this body
	smpl::collision::CollisionSpheresModelConfig config;
	if (!generateSpheresModel(shapes, transforms, config)) {
		return false;
	}

	// initialize a new spheres model
	spheres_model = new smpl::collision::CollisionSpheresModel;
	spheres_model->link_index = 0;
	spheres_model->spheres.buildFrom(config.spheres);
	ROS_DEBUG("	Spheres Model: %p", spheres_model);

	// TODO: possible make this more automatic?
	for (auto& sphere : spheres_model->spheres.m_tree) {
		sphere.parent = spheres_model;
	}

	spheres_state = new smpl::collision::CollisionSpheresState;
	spheres_state->model = spheres_model;
	spheres_state->spheres.buildFrom(spheres_state);

	return true;
}

bool Object::createVoxelsModel(
	const std::vector<shapes::ShapeConstPtr>& shapes,
	const smpl::collision::Affine3dVector& transforms)
{
	ROS_DEBUG("	Generate voxels model");

	// create configuration voxels model for this body
	smpl::collision::CollisionVoxelModelConfig config;
	generateVoxelsModel(config);

	// alloc voxels models
	voxels_model = new smpl::collision::CollisionVoxelsModel;

	// attach to the object
	voxels_model->link_index = 0;

	const double AB_VOXEL_RES = 0.01;
	voxels_model->voxel_res = AB_VOXEL_RES;

	if (!voxelizeAttachedBody(shapes, transforms, *voxels_model)) {
		ROS_ERROR("Failed to voxelize object '%d'", desc.id);
		return false;
		// TODO: anything to do in this case
	}

	voxels_state = new smpl::collision::CollisionVoxelsState;
	voxels_state->model = voxels_model;
	voxels_state->voxels = voxels_model->voxels;

	return true;
}

bool Object::generateSpheresModel(
	const std::vector<shapes::ShapeConstPtr>& shapes,
	const smpl::collision::Affine3dVector& transforms,
	smpl::collision::CollisionSpheresModelConfig& spheres_model)
{
	assert(std::all_of(shapes.begin(), shapes.end(),
			[](const shapes::ShapeConstPtr& shape) { return (bool)shape; }));
	assert(shapes.size() == transforms.size());

	ROS_DEBUG("Generate spheres model configuration");

	// TODO: reserve a special character so as to guarantee sphere uniqueness
	// here and disallow use of the special character on config-generated
	// spheres

	// TODO: yeah...
	const double object_enclosing_sphere_radius = 0.015;

	// voxelize the object
	std::vector<Eigen::Vector3d> voxels;
	for (size_t i = 0; i < shapes.size(); ++i) {
		if (!smpl::collision::VoxelizeShape(
				*shapes[i], transforms[i],
				object_enclosing_sphere_radius / std::sqrt(2),
				Eigen::Vector3d::Zero(),
				voxels))
		{
			ROS_ERROR("Failed to voxelize object shape for sphere generation");
			return false;
		}
	}

	spheres_model.autogenerate = false;
	spheres_model.link_name = std::to_string(desc.id);
	spheres_model.spheres.clear();

	for (size_t i = 0; i < voxels.size(); ++i) {
		smpl::collision::CollisionSphereConfig sphere_config;
		sphere_config.name = std::to_string(desc.id) + "/" + std::to_string(i);
		sphere_config.x = voxels[i].x();
		sphere_config.y = voxels[i].y();
		sphere_config.z = voxels[i].z();
		sphere_config.radius = object_enclosing_sphere_radius;

		spheres_model.spheres.push_back(std::move(sphere_config));
	}

	ROS_DEBUG("Generated spheres model with %zu spheres", spheres_model.spheres.size());
	return true;
}

void Object::generateVoxelsModel(
	smpl::collision::CollisionVoxelModelConfig& voxels_model)
{
	ROS_DEBUG("Generate voxels model configuration");
	voxels_model.link_name = std::to_string(desc.id);
}

bool Object::voxelizeAttachedBody(
	const std::vector<shapes::ShapeConstPtr>& shapes,
	const smpl::collision::Affine3dVector& transforms,
	smpl::collision::CollisionVoxelsModel& model) const
{
	if (shapes.size() != transforms.size()) {
		ROS_ERROR("shapes array and transforms array must have equal length");
		return false;
	}

	std::vector<Eigen::Vector3d> voxels;
	for (size_t i = 0; i < shapes.size(); ++i) {
		const shapes::Shape& shape = *shapes[i];
		const Eigen::Affine3d& transform = transforms[i];
		smpl::collision::VoxelizeShape(
				shape, transform,
				model.voxel_res, Eigen::Vector3d::Zero(), model.voxels);
	}

	return true;
}

constexpr double kFloatingPointTolerance = 1e-5;

///////////////////////////////////////////////////////////////////////////////
// ContPose
///////////////////////////////////////////////////////////////////////////////

ContPose::ContPose(double x, double y, double z, double roll, double pitch,
					double yaw) :
m_x(x), m_y(y), m_z(z),
m_roll(smpl::angles::normalize_angle_positive(roll)),
m_pitch(smpl::angles::normalize_angle_positive(pitch)),
m_yaw(smpl::angles::normalize_angle_positive(yaw))
{}

ContPose::ContPose(const DiscPose &disc_pose)
{
	m_x = DiscretisationManager::DiscXToContX(disc_pose.x());
	m_y = DiscretisationManager::DiscYToContY(disc_pose.y());
	// TODO: add DiscZToContZ, or use uniform resolution for x,y and z.
	m_z = DiscretisationManager::DiscYToContY(disc_pose.z());
	// TODO: use "DiscAngleToContAngle"
	m_roll = DiscretisationManager::DiscYawToContYaw(disc_pose.roll());
	m_pitch = DiscretisationManager::DiscYawToContYaw(disc_pose.pitch());
	m_yaw = DiscretisationManager::DiscYawToContYaw(disc_pose.yaw());
}

bool ContPose::operator==(const ContPose &other) const
{
	return fabs(m_x - other.x()) < kFloatingPointTolerance &&
			fabs(m_y - other.y()) < kFloatingPointTolerance &&
			fabs(m_z - other.z()) < kFloatingPointTolerance &&
			fabs(m_roll - other.roll()) < kFloatingPointTolerance &&
			fabs(m_pitch - other.pitch()) < kFloatingPointTolerance &&
			fabs(m_yaw - other.yaw()) < kFloatingPointTolerance;
}

bool ContPose::operator!=(const ContPose &other) const
{
	return !(*this == other);
}

Eigen::Affine3d ContPose::GetTransform() const
{
	const Eigen::AngleAxisd roll_angle(m_roll, Eigen::Vector3d::UnitX());
	const Eigen::AngleAxisd pitch_angle(m_pitch, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd yaw_angle(m_yaw, Eigen::Vector3d::UnitZ());
	const Eigen::Quaterniond quaternion = yaw_angle * pitch_angle * roll_angle;
	const Eigen::Affine3d transform(Eigen::Translation3d(m_x, m_y, m_z) * quaternion);
	return transform;
}

std::ostream &operator<< (std::ostream &stream, const ContPose &cont_pose)
{
	stream << "("
			<< cont_pose.x() << ", "
			<< cont_pose.y() << ", "
			<< cont_pose.z() << ", "
			<< cont_pose.roll() << ", "
			<< cont_pose.pitch() << ", "
			<< cont_pose.yaw()
			<< ")";
	return stream;
}

///////////////////////////////////////////////////////////////////////////////
// DiscPose
///////////////////////////////////////////////////////////////////////////////

DiscPose::DiscPose(int x, int y, int z, int roll, int pitch, int yaw) :
m_x(x), m_y(y), m_z(z),
m_roll(DiscretisationManager::NormalizeDiscreteTheta(roll)),
m_pitch(DiscretisationManager::NormalizeDiscreteTheta(pitch)),
m_yaw(DiscretisationManager::NormalizeDiscreteTheta(yaw))
{}

DiscPose::DiscPose(const ContPose &cont_pose)
{
	m_x = DiscretisationManager::ContXToDiscX(cont_pose.x());
	m_y = DiscretisationManager::ContYToDiscY(cont_pose.y());
	m_z = DiscretisationManager::ContYToDiscY(cont_pose.z());
	m_roll = DiscretisationManager::ContYawToDiscYaw(cont_pose.roll());
	m_pitch = DiscretisationManager::ContYawToDiscYaw(cont_pose.pitch());
	m_yaw = DiscretisationManager::ContYawToDiscYaw(cont_pose.yaw());
}

bool DiscPose::operator==(const DiscPose &other) const
{
	return m_x == other.x() && m_y == other.y() && m_z == other.z() &&
			m_roll == other.roll() && m_pitch == other.pitch() && m_yaw == other.yaw();
}

bool DiscPose::operator!=(const DiscPose &other) const
{
	return !(*this == other);
}

bool DiscPose::EqualsPosition(const DiscPose &other) const
{
	return m_x == other.x() && m_y == other.y() && m_z == other.z();
}

std::ostream &operator<< (std::ostream &stream, const DiscPose &disc_pose) {
	stream << "("
				<< disc_pose.x() << ", "
				<< disc_pose.y() << ", "
				<< disc_pose.z() << ", "
				<< disc_pose.roll() << ", "
				<< disc_pose.pitch() << ", "
				<< disc_pose.yaw()
				<< ")";
	return stream;
}

///////////////////////////////////////////////////////////////////////////////
// ObjectState
///////////////////////////////////////////////////////////////////////////////

ObjectState::ObjectState() :
m_id(-1), m_symmetric(false),
m_cont_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
m_disc_pose(0, 0, 0, 0, 0, 0)
{}

ObjectState::ObjectState(int id, bool symmetric, const ContPose &cont_pose) :
m_id(id), m_symmetric(symmetric),	m_cont_pose(cont_pose), m_disc_pose(cont_pose) {}

ObjectState::ObjectState(int id, bool symmetric, const DiscPose &disc_pose) :
m_id(id), m_symmetric(symmetric),	m_cont_pose(disc_pose), m_disc_pose(disc_pose) {}

bool ObjectState::operator==(const ObjectState &other) const
{
	if (m_id != other.id()) {
		return false;
	}

	if (m_symmetric != other.symmetric()) {
		return false;
	}

	if (!m_symmetric && m_disc_pose != other.disc_pose()) {
		return false;
	}

	if (m_symmetric && !m_disc_pose.EqualsPosition(other.disc_pose())) {
		return false;
	}

	return true;
}

bool ObjectState::operator!=(const ObjectState &other) const
{
	return !(*this == other);
}

std::ostream &operator<< (std::ostream &stream, const ObjectState &object_state)
{
	stream << "Object ID: " << object_state.id() << std::endl
				 << '\t' << "Symmetric: " << std::boolalpha << object_state.symmetric() <<
				 std::endl
				 << '\t' << "Disc Pose: " << object_state.disc_pose() << std::endl
				 << '\t' << "Cont Pose: " << object_state.cont_pose();
	return stream;
}

} // namespace clutter
