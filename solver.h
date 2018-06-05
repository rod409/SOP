#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <utility>
#include <unordered_map>

#include "edge.h"
#include "digraph.h"
#include "history_node.h"

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
		Solver(const Digraph& cost_graph, const Digraph& precedance_graph);
		void solve_sop();
		void print_solution();
		void nearest_neighbor();
		void set_time_limit_per_node(int limit);
		int get_static_lower_bound();
		int best_solution_cost();
		std::vector<Edge> best_solution_path();
	private:
		std::vector<Edge> solution;
		int solution_weight;
		std::vector<bool> visited_nodes;
		int last_visited_node;
		int time_limit;
		Digraph const * cost_graph;
		Digraph const * precedance_graph;
		static std::unordered_map<std::pair<std::vector<bool>, int>, HistoryNode> history;
		bool valid_node(int node);
		bool predecessors_visited(int node);
		void update_best_solution();
		void reset_solution();
		int edge_bound(int current_node);
		void backtrack(int source);
		bool better_history(int cost, int current_node);
};

#endif
