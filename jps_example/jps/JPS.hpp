#ifndef JPS_H
#define JPS_H

#include <unordered_map>
#include <cmath>

#include "Position.hpp"
#include "PathfindingNode.hpp"
#include "PriorityQueue.hpp"
#include "PathFindRule.hpp"
#include "BasePathfinder.hpp"

namespace JPS
{
	template<typename GRID>
	using PathFinder = JPS::BasePathFinder<GRID, JPS::Rule::PathFindRule_JPSPlus<GRID, true>>;
}


#endif

