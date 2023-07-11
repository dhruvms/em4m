#ifndef CONFLICTS_HPP
#define CONFLICTS_HPP

#include <pushplan/utils/types.hpp>

#include <memory>
#include <list>

namespace clutter
{

struct Constraint
{
	int m_me, m_other, m_time;
	bool m_on;
	LatticeState m_q;
};

class HighLevelNode;

struct Conflict
{
	int m_a1, m_a2, m_t, m_priority;
	bool m_on;
	LatticeState m_q1, m_q2;
	std::shared_ptr<Constraint> m_c1, m_c2;

	void InitConflict(int a1, int a2, int t, LatticeState q1, LatticeState q2, bool robot=false);
	void SetPriority(HighLevelNode* node);
	int at(std::size_t i) const;
};

bool operator<(const Conflict& a, const Conflict& b);

inline
void VecConstraint(const Constraint& constraint, std::vector<double>& c_vec)
{
	c_vec.clear();
	c_vec.push_back(constraint.m_time);
	c_vec.push_back(constraint.m_other);
	c_vec.insert(c_vec.end(), constraint.m_q.state.begin(), constraint.m_q.state.end());
}

inline
void VecConstraints(const std::list<std::shared_ptr<Constraint> >& constraints, std::vector<std::vector<double> >& c_vecs)
{
	c_vecs.clear();
	std::vector<double> c_vec;
	for (const auto& c: constraints)
	{
		VecConstraint(*(c.get()), c_vec);
		c_vecs.push_back(c_vec);
	}
}

} // namespace clutter


#endif // CONFLICTS_HPP
