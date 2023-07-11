#ifndef CBS_H_NODE_HPP
#define CBS_H_NODE_HPP

#include <pushplan/search/conflicts.hpp>
#include <pushplan/utils/constants.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <list>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <stack>
#include <queue>
#include <vector>
#include <set>

namespace clutter
{

struct DAG
{
public:
	void Clear() { m_G.clear(); };
	bool Empty() const { return m_G.empty(); };
	void Copy(const DAG& Gin) { m_G = Gin.GetDAG(); };
	bool Contains(int root) {
		return m_G.find(root) != m_G.end();
	}

	void Add(int from, int to) { m_G[from].insert(to); };
	void Remove(int from, int to);
	bool Connected(int from, int to) const;
	std::unordered_set<int> GetHigherPriorities(int root);
	std::vector<int> GetParents(int root);
	void TopologicalSort(std::stack<std::size_t>& order);
	void TopologicalSort(
		const std::vector<int>& check,
		std::queue<int>& order);

	auto GetDAG() const -> const std::unordered_map<int, std::unordered_set<int> >& {
		return m_G;
	}

private:
	std::unordered_map<int, std::unordered_set<int> > m_G;

	void traverse(
		std::size_t i,
		std::vector<bool>& v,
		std::stack<std::size_t>& order);
	void traverse(
		const std::vector<int>& check,
		std::size_t i,
		std::vector<bool>& v,
		std::queue<int>& order);
};

struct HighLevelNode
{
	struct OPENCompare
	{
		bool operator()(const HighLevelNode* p, const HighLevelNode* q) const
		{
			if (p->fval() == q->fval())
			{
				if (p->m_d == q->m_d)
				{
					if (p->m_flowtime + p->m_c == q->m_flowtime + q->m_c)
					{
						return p->m_h >= q->m_h;
					}

					return p->m_flowtime + p->m_c >= q->m_flowtime + q->m_c;
				}

				return p->m_d >= q->m_d;
			}

			return p->fval() >= q->fval();
		}
	};

	struct FOCALCompare
	{
		bool operator()(const HighLevelNode* p, const HighLevelNode* q) const
		{
			if (p->m_d == q->m_d)
			{
				if (p->fval() == q->fval())
				{
					if (p->m_flowtime + p->m_c == q->m_flowtime + q->m_c)
					{
						return p->m_h >= q->m_h;
					}

					return p->m_flowtime + p->m_c >= q->m_flowtime + q->m_c;
				}

				return p->fval() >= q->fval();
			}

			return p->m_d >= q->m_d;
		}
	};

	std::list<std::shared_ptr<Constraint> > m_constraints; // constraints to be satisfied (parent + 1)
	std::list<std::shared_ptr<Conflict> > m_conflicts, m_all_conflicts; // conflicts at this node
	std::shared_ptr<Conflict> m_conflict; // selected conflict
	std::vector<std::pair<int, Trajectory> > m_solution; // agent solutions
	DAG m_priorities;

	int m_flowtime = 0, m_makespan = 0, m_depth = 0, m_generate = 0, m_expand = 0;
	unsigned int m_g = 0, m_h = 0, m_d = 0, m_c = 0;
	bool m_h_computed = false;

	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::OPENCompare> >::handle_type m_OPEN_h;
	boost::heap::fibonacci_heap<HighLevelNode*, boost::heap::compare<HighLevelNode::FOCALCompare> >::handle_type m_FOCAL_h;

	HighLevelNode* m_parent = nullptr;
	std::vector<HighLevelNode*> m_children;
	int m_replanned;

	~HighLevelNode();

	void clear() {
		m_conflicts.clear();
		m_priorities.Clear();
	};
	unsigned int fval() const { return this->m_g + this->m_h; };
	unsigned int fhatval() const { return this->m_flowtime + this->m_c; };

	void recalcFlowtime();
	void recalcMakespan();
	void recalcG(const std::vector<unsigned int>& min_fs);
	void updateDistanceToGo();
	void updateCostToGo();
	void computeH();

	void PrintDebug() {
		SMPL_INFO("\tg = %u, h = %u, d = %u, c = %u", m_g, m_h, m_d, m_c);
		SMPL_INFO("\t\tflowtime = %d, makespan = %d, depth = %d, generated = %d, expanded = %d", m_flowtime, m_makespan, m_depth, m_generate, m_expand);
		SMPL_INFO("\t\t\tall_conflicts = %d, conflicts = %d", m_all_conflicts.size(), m_conflicts.size());
	}
};

struct LowLevelNode
{
	int call_number;
	int state_id;
	unsigned int g = 0, h = 0, f = 0, h_c = 0;
	LowLevelNode* bp = nullptr;
	bool closed = false;

	struct OPENCompare
	{
		bool operator()(const LowLevelNode* p, const LowLevelNode* q) const
		{
			if (p->f == q->f)
			{
				if (p->h == q->h)
				{
					return rand() % 2;
				}

				return p->h >= q->h;
			}

			return p->f >= q->f;
		}
	};

	struct FOCALCompare
	{
		bool operator()(const LowLevelNode* p, const LowLevelNode* q) const
		{
			if (p->h_c == q->h_c)
			{
				if (p->f == q->f)
				{
					if (p->h == q->h)
					{
						return rand() % 2;
					}

					return p->h >= q->h;
				}

				return p->f >= q->f;
			}

			return p->h_c >= q->h_c;
		}
	};

	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::OPENCompare> >::handle_type m_OPEN_h;
	boost::heap::fibonacci_heap<LowLevelNode*, boost::heap::compare<LowLevelNode::FOCALCompare> >::handle_type m_FOCAL_h;
};

} // namespace clutter


#endif // CBS_H_NODE_HPP
