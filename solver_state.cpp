#include <vector>

#include "edge.h"
#include "solver_state.h"

using std::vector;

SolverState::SolverState(Edge next_edge, std::vector<Edge> path, int cost){
	this->next_edge = next_edge;
	this->path = path;
	this->cost = cost;
}

SolverState::SolverState(const SolverState& state){
	this->next_edge = state.next_edge;
	this->path = state.path;
	this->cost = state.cost;
}

