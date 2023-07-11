#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <pushplan/utils/bullet_sim.hpp>
#include <pushplan/utils/constants.hpp>
#include <pushplan/utils/types.hpp>

#include <smpl/time.h>
#include <sbpl_collision_checking/shapes.h> // MeshShape etc.
#include <geometric_shapes/shape_operations.h> // constructShapeFromMsg, createMeshFromResource
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <sys/stat.h>

namespace clutter
{

inline
bool FileExists(const std::string& filename)
{
	struct stat buf;
	if (stat(filename.c_str(), &buf) != -1)
	{
		return true;
	}
	return false;
}

static double GetTime()
{
	using namespace smpl;
	return to_seconds(clock::now().time_since_epoch());
}

template <typename T>
inline
int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

inline
double dot(const State& a, const State& b)
{
	assert(a.size() == b.size());
	double val = 0.0;
	for (size_t i = 0; i < a.size(); ++i) {
		val += a.at(i) * b.at(i);
	}
	if (std::abs(val) < 1e-6) {
		val = 0.0;
	}
	return val;
}

inline
State vector2D(const State& from, const State& to)
{
	assert(from.size() == to.size());
	State v(from.size(), 0.0);
	v.at(0) = to.at(0) - from.at(0);
	v.at(1) = to.at(1) - from.at(1);

	return v;
}

template <class T>
static auto ParseMapFromString(const std::string& s)
	-> std::unordered_map<std::string, T>
{
	std::unordered_map<std::string, T> map;
	std::istringstream ss(s);
	std::string key;
	T value;
	while (ss >> key >> value) {
		map.insert(std::make_pair(key, value));
	}
	return map;
}

// Construct a CollisionShape from a Mesh. Deep copies of the vertex and index
// data are made. Both the returned mesh shape and the associated vertex and
// index are expected to be freed by the caller.
// Note: function copied from sbpl_collision_checking/conversions.cpp
static auto ConstructShapeFromMsg(const shape_msgs::Mesh& mesh)
	-> smpl::collision::MeshShape*
{
	auto vertices = new double[3 * mesh.vertices.size()];
	auto triangles = new std::uint32_t[3 * mesh.triangles.size()];

	for (size_t i = 0; i < mesh.vertices.size(); ++i) {
		vertices[3 * i + 0] = mesh.vertices[i].x;
		vertices[3 * i + 1] = mesh.vertices[i].y;
		vertices[3 * i + 2] = mesh.vertices[i].z;
	}

	for (size_t i = 0; i < mesh.triangles.size(); ++i) {
		triangles[3 * i + 0] = mesh.triangles[i].vertex_indices[0];
		triangles[3 * i + 1] = mesh.triangles[i].vertex_indices[1];
		triangles[3 * i + 2] = mesh.triangles[i].vertex_indices[2];
	}

	auto* m = new smpl::collision::MeshShape;
	m->vertices = vertices;
	m->triangles = triangles;
	m->vertex_count = mesh.vertices.size();
	m->triangle_count = mesh.triangles.size();
	return m;
}

static auto ConvertSTLToMeshShape(
	const std::string& stl_file)
	-> smpl::collision::MeshShape*
{
	shapes::Mesh* mesh = nullptr;
	mesh = shapes::createMeshFromResource(stl_file, Eigen::Vector3d::Constant(1.0));

	if (!mesh) {
		ROS_ERROR("Failed to convert resource file to mesh!");
		return nullptr;
	}

	shape_msgs::Mesh mesh_msg;
	shapes::ShapeMsg shape_msg = mesh_msg;
	if (shapes::constructMsgFromShape(mesh, shape_msg)) {
		mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
	}
	else {
		ROS_ERROR("Failed to get Mesh msg!");
		return nullptr;
	}

	return ConstructShapeFromMsg(mesh_msg);
}

// Construct a shapes::Shape from a CollisionShape. The CollisionShape must
// be of a known type. The data associated with the CollisionShape is deep
// copied to be managed by the returned shapes::Shape.
static auto MakeROSShape(const smpl::collision::CollisionShape* shape)
	-> shapes::ShapeConstPtr
{
	switch (shape->type) {
	case smpl::collision::ShapeType::Sphere:
	{
		auto* s = static_cast<const smpl::collision::SphereShape*>(shape);
		return shapes::ShapeConstPtr(new shapes::Sphere(s->radius));
	}
	case smpl::collision::ShapeType::Cylinder:
	{
		auto* s = static_cast<const smpl::collision::CylinderShape*>(shape);
		return shapes::ShapeConstPtr(new shapes::Cylinder(s->radius, s->height));
	}
	case smpl::collision::ShapeType::Cone:
	{
		auto* s = static_cast<const smpl::collision::ConeShape*>(shape);
		return shapes::ShapeConstPtr(new shapes::Cone(s->radius, s->height));
	}
	case smpl::collision::ShapeType::Box:
	{
		auto* s = static_cast<const smpl::collision::BoxShape*>(shape);
		return shapes::ShapeConstPtr(new shapes::Box(s->size[0], s->size[1], s->size[2]));
	}
	case smpl::collision::ShapeType::Plane:
	{
		auto* s = static_cast<const smpl::collision::PlaneShape*>(shape);
		return shapes::ShapeConstPtr(new shapes::Plane(s->a, s->b, s->c, s->d));
	}
	case smpl::collision::ShapeType::Mesh:
	{
		auto* s = static_cast<const smpl::collision::MeshShape*>(shape);

		auto vertices = new double[3 * s->vertex_count];
		auto triangles = new std::uint32_t[3 * s->triangle_count];

		std::copy(s->vertices, s->vertices + 3 * s->vertex_count, vertices);
		std::copy(s->triangles, s->triangles + 3 * s->triangle_count, triangles);

		auto ms = new shapes::Mesh;
		ms->vertices = vertices;
		ms->vertex_count = s->vertex_count;
		ms->triangles = triangles;
		ms->triangle_count = s->triangle_count;
		return shapes::ShapeConstPtr(ms);
	}
	case smpl::collision::ShapeType::OcTree:
	{
		auto* s = static_cast<const smpl::collision::OcTreeShape*>(shape);

		auto octree = decltype(shapes::OcTree().octree)(
				new octomap::OcTree(*s->octree));

		return shapes::ShapeConstPtr(new shapes::OcTree(octree));
	}
	default:
		return NULL;
	}
}

inline
int armId(const sensor_msgs::JointState& state)
{
	int arm;
	for (const auto& name : state.name)
	{
		if (name.find("r_") == 0)
		{
			arm = 1;
			break;
		}

		else if (name.find("l_") == 0)
		{
			arm = 0;
			break;
		}
	}
	return arm;
}

inline
bool setupSim(BulletSim* sim, const sensor_msgs::JointState& state, const int& ooi_id)
{
	if (!sim->SetRobotState(state))
	{
		ROS_ERROR("Failed to set start state!");
		return false;
	}

	int arm = armId(state);
	if (!sim->ResetArm(1 - arm))
	{
		ROS_ERROR("Failed to reset other arm!");
		return false;
	}

	int removed;
	if (!sim->CheckScene(arm, removed))
	{
		ROS_ERROR("Failed to check for start state collisions with objects!");
		return false;
	}

	if (!sim->ResetScene()) {
		ROS_ERROR("Failed to reset scene objects!");
		return false;
	}

	if (!sim->SetColours(ooi_id))
	{
		ROS_ERROR("Failed to set object colours in scene!");
		return false;
	}

	return true;
}

template <typename Mat_t>
Mat_t MoorePenrosePInv(const Mat_t& mat, double epsilon=std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD<Mat_t> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

}

#endif // HELPERS_HPP
