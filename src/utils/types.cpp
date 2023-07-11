#include <pushplan/utils/types.hpp>

namespace clutter
{

// bool operator==(
// 	const std::map<Coord, int, coord_compare> &lhs,
// 	const std::map<Coord, int, coord_compare> &rhs)
// {
// 	return lhs.size() == rhs.size() &&
// 			std::equal(lhs.begin(), lhs.end(), rhs.begin(),
// 					  [] (const auto &a, const auto &b) { return a.first == b.first; });
// }

bool operator==(const LatticeState& a, const LatticeState& b)
{
	return (
		a.coord == b.coord &&
		a.t == b.t
	);
}

bool operator!=(const LatticeState& a, const LatticeState& b)
{
	return !(a == b);
}

bool operator==(
	const std::vector<std::pair<int, Trajectory> >& a,
	const std::vector<std::pair<int, Trajectory> >& b)
{
	if (a.size() != b.size()) {
		return false;
	}

	for (auto it_a = a.begin(), it_b = b.begin();
							it_a != a.end() && it_b != b.end(); ++it_a, ++it_b)
	{
		if (it_a->first != it_b->first) {
			return false;
		}

		if (it_a->second.size() != it_b->second.size()) {
			return false;
		}

		for (auto wpit_a = it_a->second.begin(), wpit_b = it_b->second.begin();
					wpit_a != it_a->second.end() && wpit_b != it_b->second.end();
						++wpit_a, ++wpit_b)
		{
			if (*wpit_a != *wpit_b) {
				return false;
			}
		}
	}

	return true;
}

bool operator!=(
	const std::vector<std::pair<int, Trajectory> >& a,
	const std::vector<std::pair<int, Trajectory> >& b)
{
	return !(a == b);
}

} // namespace clutter
