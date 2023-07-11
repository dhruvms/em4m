#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <pushplan/agents/object.hpp>
#include <pushplan/utils/types.hpp>
#include <pushplan/utils/helpers.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <cmath>

namespace clutter
{

bool PointInRectangle(const State& P, const std::vector<State>& R);
bool LineSegCircleIntersect(
	const State& C, double r,
	const State& A, const State& B);
bool RectanglesIntersect(
	const std::vector<State>& R1,
	const std::vector<State>& R2);
void LineLineIntersect(
	const State& l1a, const State& l1b,
	const State& l2a, const State& l2b,
	State& p);

bool is_between_ordered(double val, double lb, double ub);
bool overlaps(double min1, double max1, double min2, double max2);
void test_SAT(const State& axis, const std::vector<State>& rect, double& rmin, double& rmax);

void MakeObjectRectangle(const ObjectDesc& o, std::vector<State>& rect);
void GetRectObjAtPt(const State& p,	const ObjectDesc& o, std::vector<State>& rect);

double PtDistFromLine(const State& p, const State& A, const State& B);

void ArmRectObj(
	const State& F1, const State& F2,
	const double& b,
	ObjectDesc& o);

template<typename T>
inline
double EuclideanDist(const std::vector<T>& p1, const std::vector<T>& p2)
{
	// assert(p1.size() == p2.size());
	double val = 0.0;
	for (size_t i = 0; i < p1.size(); ++i) {
		val += std::pow(p1.at(i) - p2.at(i), 2);
	}
	return std::sqrt(val);
}

template<typename T>
inline
double ManhattanDist(const std::vector<T>& p1, const std::vector<T>& p2)
{
	// assert(p1.size() == p2.size());
	double val = 0.0;
	for (size_t i = 0; i < p1.size(); ++i) {
		val += std::abs(p1.at(i) - p2.at(i));
	}
	return val;
}

} // namespace clutter


#endif // GEOMETRY_HPP
