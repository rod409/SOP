#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <utility>

#include "edge.h"
#include "digraph.h"
#include "history_node.h"
#include "hash_map.h"
#include "solver_state.h"
#include "hungarian.h"

namespace std
{
    template<>
    struct hash<std::pair<vector<bool>, int>> {
        size_t operator () (std::pair<vector<bool>, int> const& p) const {
            return (std::hash<vector<bool>>()(p.first) ^ std::hash<int>()(p.second));
        }
    };
}

class Solver {
	public:
		Solver(Digraph const * cost_graph, Digraph const * precedance_graph);
		Solver(Digraph const * cost_graph, Digraph const * precedance_graph, Hungarian h);
		void solve_sop_parallel(int nub_threads);
		void solve_sop(SolverState first_visit);
		void print_solution();
		void nearest_neighbor();
		void set_time_limit_per_node(int limit);
		void set_hash_size(size_t size);
		int get_static_lower_bound();
		int best_solution_cost();
		std::vector<Edge> best_solution_path();
	private:
		std::vector<Edge> solution;
		int solution_weight;
		std::vector<bool> visited_nodes;
		int last_visited_node;
		Digraph const * cost_graph;
		Digraph const * precedance_graph;
		static HashMap<std::pair<std::vector<bool>, int>, HistoryNode> history;
		static int thread_count;
		bool valid_node(int node);
		bool predecessors_visited(int node);
		void update_best_solution();
		void reset_solution(SolverState state);
		int edge_bound(int current_node);
		void backtrack(int source);
		bool better_history(int cost, int current_node);
		void split_visits();
		static vector<vector<int>> cost_matrix;
		static int max_edge_weight;
		Hungarian hungarian_solver;
};

#endif
