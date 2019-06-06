#include <limits>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <atomic>

#include <iostream>

#include "solver.h"
#include "edge.h"
#include "digraph.h"
#include "history_node.h"
#include "hash_map.h"
#include "hungarian.h"

using std::vector;
using std::pair;

static int best_solution = std::numeric_limits<int>::max();
static vector<Edge> best_solution_nodes;
static int static_lower_bound = 0;

static int time_limit = 100;
static std::chrono::time_point<std::chrono::system_clock> start_time;
HashMap<pair<vector<bool>, int>, HistoryNode> Solver::history;
int Solver::max_edge_weight = 0;
vector<vector<int>> Solver::cost_matrix;

Solver::Solver(Digraph const * cost_graph, Digraph const * precedance_graph){
	this->cost_graph = cost_graph;
	this->precedance_graph = precedance_graph;
	visited_nodes.assign(cost_graph->node_count(), false);
	solution_weight = 0;
	cost_matrix = this->cost_graph->dense_hungarian();
	max_edge_weight = this->cost_graph->get_max_edge_weight();
	hungarian_solver = Hungarian(cost_graph->node_count(), max_edge_weight, cost_matrix);
	enumerated_nodes = 0;
	bound_calculations = 0;
}

void Solver::set_time_limit(int limit, bool per_node){
	if(per_node){
	    time_limit = limit*cost_graph->node_count();
	} else {
	    time_limit = limit;
	}
}

void Solver::set_hash_size(size_t size){
	size_t bit_vector_mem = sizeof(visited_nodes) + std::max((size_t)std::ceil((float)cost_graph->node_count()/8), (size_t)32);
	history.set_size(size, bit_vector_mem);
	
}

void Solver::solve_sop(){
	std::chrono::time_point<std::chrono::system_clock> start_time, current_time;
	start_time = std::chrono::system_clock::now();
	int solution_size = cost_graph->node_count();	
	for(int i = 0; i < cost_graph->node_count(); ++i){
		vector<Edge> st;
		reset_solution();
		if(predecessors_visited(i)){
			Edge start(i, i, 0);
			st.push_back(start);
			while(!st.empty()){
				Edge last_edge = st.back();
				st.pop_back();
				solution.push_back(last_edge);
				solution_weight += last_edge.weight;
				
				visited_nodes[last_edge.dest] = true;
				last_visited_node = last_edge.dest;
				hungarian_solver.fix_row(last_edge.source, last_edge.dest);
		        hungarian_solver.fix_column(last_edge.dest, last_edge.source);
				++enumerated_nodes;
				if(solution.size() == solution_size){
                   update_best_solution();
                   if(!st.empty()){
                           backtrack(st.back().source);
                   }
                } else {
					const vector<Edge>& sorted_outgoing = cost_graph->sorted_adj_outgoing(last_edge.dest);
					vector<pair<Edge, int>> priority_edges;
				    for(int i = sorted_outgoing.size() - 1; i >= 0; --i){
					    if(valid_node(sorted_outgoing[i].dest)){
						    visited_nodes[sorted_outgoing[i].dest] = true;
						    solution_weight += sorted_outgoing[i].weight;
						    solution.push_back(sorted_outgoing[i]);
						    pair<Edge, int> node_pair(sorted_outgoing[i], get_lower_bound(solution_weight, sorted_outgoing[i].dest, visited_nodes));
						    visited_nodes[sorted_outgoing[i].dest] = false;
						    hungarian_solver.undue_row(sorted_outgoing[i].source, sorted_outgoing[i].dest);
		                    hungarian_solver.undue_column(sorted_outgoing[i].dest, sorted_outgoing[i].source);
						    solution_weight -= sorted_outgoing[i].weight;
						    solution.pop_back();
						    if(node_pair.second < best_solution){
						        priority_edges.push_back(node_pair);
						    }
					    }
				    }
				    if(priority_edges.size() > 0){
				        std::sort(priority_edges.begin(), priority_edges.end(), EdgeBoundCompare);
				        for(int i = 0; i < priority_edges.size(); ++i){
				            st.push_back(priority_edges[i].first);
				        }
				    } else {
				        if(!st.empty()){
					        backtrack(st.back().source);
				        }
				    }
				}
				current_time = std::chrono::system_clock::now();
			    std::chrono::duration<double> elapsed_time = current_time - start_time;
			    if(elapsed_time.count() > time_limit){
				    break;
			    }
			}
		}
	}
}

void Solver::backtrack(int source){
	while(solution.back().dest != source){
		Edge e = solution.back();
		solution.pop_back();
		visited_nodes[e.dest] = false;
		hungarian_solver.undue_row(e.source, e.dest);
		hungarian_solver.undue_column(e.dest, e.source);
		last_visited_node = e.source;
		solution_weight -= e.weight;
	}
}

bool Solver::valid_node(int node){
	return !visited_nodes[node] && predecessors_visited(node);
}

bool Solver::predecessors_visited(int node){
	bool all_visited = true;
	for(Edge e: precedance_graph->adj_outgoing(node)){
		if(!visited_nodes[e.dest]){
			all_visited = false;
			break;
		}
	}
	return all_visited;
}

void Solver::update_best_solution(){
	if(solution_weight < best_solution){
		best_solution = solution_weight;
		best_solution_nodes = solution;
	}
}

void Solver::reset_solution(){
	solution.clear();
	solution_weight = 0;
	visited_nodes.assign(cost_graph->node_count(), false);
	hungarian_solver = Hungarian(cost_matrix.size(), max_edge_weight, cost_matrix);
	hungarian_solver.start();
}

int Solver::get_static_lower_bound(){
	return static_lower_bound;
}

int Solver::best_solution_cost(){
	return best_solution;
}

vector<Edge> Solver::best_solution_path(){
	return best_solution_nodes;
}

int Solver::edge_bound(int current_node){
	int largest_out_weight = 0;
	int outgoing_edge_weights = 0;
	int largest_in_weight = 0;
	int incoming_edge_weights = 0;
	for(int i = 0; i < cost_graph->node_count(); ++i){
		if((!visited_nodes[i] || i == current_node)){
			int min_out_weight = -1;
			for(const Edge& e : cost_graph->sorted_adj_outgoing(i)){
				if(!visited_nodes[e.dest]){
					min_out_weight = e.weight;
					break;
				}
			}
			if(min_out_weight >= 0){
				outgoing_edge_weights += min_out_weight;
			} else {
			    min_out_weight = max_edge_weight;
			    outgoing_edge_weights += min_out_weight;
			}
			if(min_out_weight > largest_out_weight){
				largest_out_weight = min_out_weight;
			}
			if(i != current_node){
				int min_in_weight = -1;
				for(const Edge& e : cost_graph->sorted_adj_incoming(i)){
					if((!visited_nodes[e.source] || e.source == current_node)){
						min_in_weight = e.weight;
						break;
					}
				}
				if(min_in_weight >= 0){
					incoming_edge_weights += min_in_weight;
				} else {
				    min_in_weight = max_edge_weight;
				    incoming_edge_weights += min_in_weight;
				}
				if(min_in_weight > largest_in_weight){
					largest_in_weight = min_in_weight;
				}
			}
			
		}
	}
	int outgoing_bound = solution_weight + outgoing_edge_weights - largest_out_weight;
	int incoming_bound = solution_weight + incoming_edge_weights;
	return std::max(outgoing_bound, incoming_bound);
}

int Solver::get_lower_bound(int cost, int current_node, vector<bool> visits){
	pair<vector<bool>, int> history_pair = pair<vector<bool>, int>(visits, current_node);
	auto p = history.find(history_pair);
	int lower_bound = std::numeric_limits<int>::max();
	if(p != history.end()){
		if(p->second.prefix_cost > cost){
			int improvement = p->second.prefix_cost - cost;
			lower_bound = p->second.lower_bound - improvement;
			if(lower_bound < best_solution){
                Edge last_edge = solution.back();
				hungarian_solver.fix_row(last_edge.source, last_edge.dest);
				hungarian_solver.fix_column(last_edge.dest, last_edge.source);
				p->second.prefix_cost = cost;
				p->second.lower_bound = lower_bound;
			} else {
				p->second.prefix_cost = cost;
				p->second.lower_bound = lower_bound;
			}
		}
	} else {	
		Edge last_edge = solution.back();
		hungarian_solver.fix_row(last_edge.source, last_edge.dest);
		hungarian_solver.fix_column(last_edge.dest, last_edge.source);
		hungarian_solver.solve_dynamic();
		lower_bound = hungarian_solver.get_matching_cost()/2;
		++bound_calculations;
		history.put(history_pair, HistoryNode(cost, lower_bound));
	}
	return lower_bound;
}

void Solver::nearest_neighbor(){
	Edge current_node(0,0,0);
	static_lower_bound = hungarian_solver.start()/2;
	reset_solution();
	visited_nodes[0] = true;
	solution.push_back(current_node);
	bool continue_search = true;
	while(continue_search){
		for(const Edge& e : cost_graph->sorted_adj_outgoing(current_node.dest)){
			if(!visited_nodes[e.dest] && predecessors_visited(e.dest)){
				solution.push_back(e);
				solution_weight += e.weight;
				visited_nodes[e.dest] = true;
				current_node = e;
				break;
			}
		}
		if(solution.size() == cost_graph->node_count()){
			continue_search = false;
		}
	}
	if(solution_weight < best_solution){
		best_solution = solution_weight;
		best_solution_nodes = solution;
	}
}

unsigned long int Solver::get_enumerated_nodes(){
    return enumerated_nodes;
}

unsigned long int Solver::get_bound_calculations(){
    return bound_calculations;
}

void Solver::set_initial_solution(vector<Edge> path, int cost){
    if(cost < best_solution){
        best_solution = cost;
	    best_solution_nodes = path;
	}
}


