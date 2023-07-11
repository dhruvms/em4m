#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <vector>
#include <map>
#include <string>

namespace clutter
{

enum class LowLevelConflictHeuristic
{
	ZERO,
	BINARY,
	COUNT,
	OURS,
	LLHC_TYPES
};
enum class HighLevelConflictHeuristic
{
	ZERO,
	CONFLICT_COUNT,
	AGENT_COUNT,
	AGENT_PAIRS,
	HLHC_TYPES
};
enum class ConflictPrioritisation
{
	RANDOM,
	EARLIEST,
	CONFLICTS,
	CP_TYPES
};
enum class MAPFAlgo
{
	WHCA,
	VCBS,
	ECBS,
	CBSWP,
	PBS,
	OURS,
	ALGO_TYPES
};

extern bool FRIDGE;

extern double MAPF_PLANNING_TIME;
extern double RES;
extern double GOAL_THRESH;

extern int WINDOW;
extern int GRID;

extern double R_SPEED;

extern bool SAVE;
extern bool CC_2D;
extern bool CC_3D;

extern double DF_RES;

extern LowLevelConflictHeuristic LLHC;
extern HighLevelConflictHeuristic HLHC;
extern ConflictPrioritisation CP;
extern MAPFAlgo ALGO;

extern const std::vector<int> YCB_OBJECTS;
extern const std::map<int, std::string> YCB_OBJECT_NAMES;
extern const std::map<int, std::vector<double>> YCB_OBJECT_DIMS;

extern const std::map<int, std::string> EXEC_TRAJ_FAIL;

extern double DEG5;
extern double LOG2PI;

extern int SAMPLES;

} // namespace clutter


#endif // CONSTANTS_HPP
