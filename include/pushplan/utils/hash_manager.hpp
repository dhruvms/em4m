#ifndef MAMO_SEARCH_ENV_HPP
#define MAMO_SEARCH_ENV_HPP

#include <pushplan/search/mamo_node.hpp>

#include <boost/heap/fibonacci_heap.hpp>

#include <unordered_map>

namespace clutter
{

template<class HashFunction, class KeyEquals>
class HashManager
{
public:
	HashManager();

	size_t Size() const;
	bool Exists(MAMONode *hashable_node) const;
	bool Exists(unsigned int node_id) const;

	MAMONode* GetState(unsigned int node_id) const;
	unsigned int GetStateID(MAMONode *hashable_node) const;
	unsigned int GetStateIDForceful(MAMONode *hashable_node);
	void UpdateState(MAMONode *hashable_node);
	void InsertState(MAMONode *hashable_node, int state_id);

	const std::unordered_map<MAMONode*, unsigned int, HashFunction, KeyEquals>& GetStateMappings() const {
		return m_node_to_node_id;
	};

	void Reset();
	// void Print() const;

private:
	std::unordered_map<MAMONode*, unsigned int, HashFunction, KeyEquals> m_node_to_node_id;
	std::unordered_map<unsigned int, MAMONode*> m_node_id_to_node;
};

/////////////////////////////////////
// Template Inline Implementations //
/////////////////////////////////////

template<class HashFunction, class KeyEquals>
HashManager<HashFunction, KeyEquals>::HashManager() {};

template<class HashFunction, class KeyEquals>
size_t HashManager<HashFunction, KeyEquals>::Size() const
{
	return m_node_to_node_id.size();
}

template<class HashFunction, class KeyEquals>
bool HashManager<HashFunction, KeyEquals>::Exists(MAMONode *hashable_node) const
{
	return m_node_to_node_id.find(hashable_node) != m_node_to_node_id.end();
}

template<class HashFunction, class KeyEquals>
bool HashManager<HashFunction, KeyEquals>::Exists(unsigned int node_id) const
{
	return m_node_id_to_node.find(node_id) != m_node_id_to_node.end();
}

template<class HashFunction, class KeyEquals>
MAMONode* HashManager<HashFunction, KeyEquals>::GetState(unsigned int node_id) const
{
	const auto it = m_node_id_to_node.find(node_id);

	if (it == m_node_id_to_node.end()) {
		throw std::runtime_error("State ID does not exist in hash table!");
	}

	return it->second;
}

template<class HashFunction, class KeyEquals>
unsigned int HashManager<HashFunction, KeyEquals>::GetStateID(MAMONode *hashable_node) const
{
	const auto it = m_node_to_node_id.find(hashable_node);

	if (it == m_node_to_node_id.end()) {
		throw std::runtime_error("MAMONode does not exist in hash table!");
	}

	return it->second;
}

template<class HashFunction, class KeyEquals>
unsigned int HashManager<HashFunction, KeyEquals>::GetStateIDForceful(MAMONode *hashable_node)
{
	const auto it = m_node_to_node_id.find(hashable_node);

	if (it != m_node_to_node_id.end()) {
		return it->second;
	}

	const int new_state_id = this->Size();
	m_node_to_node_id[hashable_node] = new_state_id;
	m_node_id_to_node.insert(std::make_pair(new_state_id, hashable_node));
	return new_state_id;
}

template<class HashFunction, class KeyEquals>
void HashManager<HashFunction, KeyEquals>::Reset()
{
	m_node_to_node_id.clear();
	m_node_id_to_node.clear();
}


template<class HashFunction, class KeyEquals>
void HashManager<HashFunction, KeyEquals>::UpdateState(MAMONode *hashable_node)
{
	const auto it = m_node_to_node_id.find(hashable_node);

	if (it == m_node_to_node_id.end())	{
		throw std::runtime_error("Asked to update a non-existent MAMONode!");
	}

	const unsigned int old_state_id = it->second;

	m_node_to_node_id.erase(it);
	m_node_to_node_id[hashable_node] = old_state_id;

	auto node_id_to_node_iterator = m_node_id_to_node.find(old_state_id);
	node_id_to_node_iterator->second = hashable_node;
}

template<class HashFunction, class KeyEquals>
void HashManager<HashFunction, KeyEquals>::InsertState(MAMONode *hashable_node, int state_id) {
	const auto it = m_node_to_node_id.find(hashable_node);

	if (it != m_node_to_node_id.end()) {
		throw std::runtime_error("Asked to insert an already existent MAMONode!");
	}

	m_node_to_node_id[hashable_node] = state_id;
	m_node_id_to_node.insert(std::make_pair(state_id, hashable_node));
}

} // namespace clutter


#endif // MAMO_SEARCH_ENV_HPP
