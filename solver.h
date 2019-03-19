#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <utility>
#include <unordered_map>

#include "edge.h"
#include "digraph.h"
#include "history_node.h"
#include "hash_map.h"
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
		void solve_sop();
		void print_solution();
		void nearest_neighbor();
		void set_time_limit(int limit, bool per_node);
		void set_hash_size(size_t size);
		int get_static_lower_bound();
		int best_solution_cost();
		std::vector<Edge> best_solution_path();
		unsigned long int get_enumerated_nodes();
		unsigned long int get_bound_calculations();
	private:
		std::vector<Edge> solution;
		int solution_weight;
		std::vector<bool> visited_nodes;
		int last_visited_node;
		int time_limit;
		Digraph const * cost_graph;
		Digraph const * precedance_graph;
		static HashMap<std::pair<std::vector<bool>, int>, HistoryNode> history;
		bool valid_node(int node);
		bool predecessors_visited(int node);
		void update_best_solution();
		void reset_solution();
		int edge_bound(int current_node);
		void backtrack(int source);
		bool better_history(int cost, int current_node);
		static vector<vector<int>> cost_matrix;
		static int max_edge_weight;
		Hungarian hungarian_solver;
		int thread_id;
		unsigned long int enumerated_nodes;
		unsigned long int bound_calculations;
};

#endif
