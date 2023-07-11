#include <pushplan/search/conflicts.hpp>
#include <pushplan/search/cbs_nodes.hpp>

namespace clutter
{

void Conflict::InitConflict(int a1, int a2, int t, LatticeState q1, LatticeState q2, bool robot)
{
	m_a1 = a1;
	m_a2 = a2;
	m_t = t;
	m_q1 = q1;
	m_q2 = q2;

	m_c1 = std::make_shared<Constraint>();
	m_c1->m_me = m_a1;
	m_c1->m_other = m_a2;
	m_c1->m_time = m_t;
	m_c1->m_q = q2;

	m_c2 = std::make_shared<Constraint>();
	m_c2->m_me = m_a2;
	m_c2->m_other = robot ? m_a2 : m_a1;
	m_c2->m_time = m_t;
	m_c2->m_q = robot ? q2 : q1;
}

void Conflict::SetPriority(HighLevelNode* node)
{
	switch (CP)
	{
		case ConflictPrioritisation::RANDOM:
		{
			m_priority = 0;
			break;
		}
		case ConflictPrioritisation::EARLIEST:
		{
			m_priority = m_t;
			break;
		}
		case ConflictPrioritisation::CONFLICTS:
		{
			int count = 0;
			for (const auto& c : node->m_conflicts)
			{
				if (c->m_a1 == m_a1 || c->m_a2 == m_a1
						|| c->m_a1 == m_a2 || c->m_a2 == m_a2) {
					++count;
				}
			}
			m_priority = count;
			break;
		}
	}
}

int Conflict::at(std::size_t i) const
{
	if (i == 0) {
		return m_a1;
	}
	else if (i == 1) {
		return m_a2;
	}
	else {
		assert(false);
		return -1;
	}
}

bool operator<(const Conflict& a, const Conflict& b)
{
	if (a.m_priority == b.m_priority) {
		return rand() % 2;
	}
	return a.m_priority > b.m_priority;
}

} // namespace clutter
