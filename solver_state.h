#ifndef SOLVER_STATE_H
#define SOLVER_STATE_H

#include <vector>

#include "edge.h"

class SolverState{
	public:
		SolverState(Edge next_edge, std::vector<Edge> path, int cost);
		SolverState(const SolverState& state);
		Edge next_edge;
		std::vector<Edge> path;
		int cost;
};

#endif
