#include <limits>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>
#include <cmath>

#include "solver.h"
#include "edge.h"
#include "digraph.h"
#include "history_node.h"
#include "hash_map.h"
#include "solver_state.h"

using std::vector;
using std::pair;
using std::thread;
using std::mutex;
using std::priority_queue;

static int best_solution = std::numeric_limits<int>::max();
static vector<Edge> best_solution_nodes;
static int static_lower_bound = 0;
static int time_limit = 100;
static std::chrono::time_point<std::chrono::system_clock> start_time;
static mutex best_solution_mutex;
static priority_queue<SolverState, vector<SolverState>, SolverStateCompare> first_visits;
static mutex first_visits_mutex;
static int counter = 0;

HashMap<pair<vector<bool>, int>, HistoryNode> Solver::history;
int Solver::thread_count = 0;
int Solver::max_edge_weight = 0;

Solver::Solver(Digraph const * cost_graph, Digraph const * precedance_graph){
	this->cost_graph = cost_graph;
	this->precedance_graph = precedance_graph;
	visited_nodes.assign(cost_graph->node_count(), false);
	solution_weight = 0;
}

Solver::Solver(Digraph const * cost_graph, Digraph const * precedance_graph, Hungarian h){
	this->cost_graph = cost_graph;
	this->precedance_graph = precedance_graph;
	visited_nodes.assign(cost_graph->node_count(), false);
	solution_weight = 0;
}

void Solver::set_time_limit_per_node(int limit){
	time_limit = limit*cost_graph->node_count();
}

void Solver::set_hash_size(size_t size){
	size_t bit_vector_mem = sizeof(visited_nodes) + std::max((size_t)std::ceil((float)cost_graph->node_count()/8), (size_t)32);
	history.set_size(size, bit_vector_mem);
	
}

void Solver::solve_sop_parallel(int num_threads){
	start_time = std::chrono::system_clock::now();
	thread_count = num_threads;
	for(int i = 0; i < cost_graph->node_count(); ++i){
		solution.clear();
		solution.push_back(Edge(i, i, 0));
		reset_solution(SolverState(Edge(i, i, 0), vector<Edge>(1, Edge(i, i, 0)), 0));
		for(const Edge& e : cost_graph->adj_outgoing(i)){
			if(valid_node(e.dest)){
				first_visits.push(SolverState(e, vector<Edge>(1, Edge(i, i, 0)), 0));
			}
		}
		if(!first_visits.empty()){
			vector<thread> solver_threads(thread_count);
			vector<Solver> solvers;
			first_visits_mutex.lock();
			for(int j = 0; j < thread_count; ++j){
				solvers.push_back(Solver(cost_graph, precedance_graph));
				if(first_visits.size() < thread_count){
					split_visits();
				}
				if(!first_visits.empty()){
					SolverState first_visit = first_visits.top();
					first_visits.pop();
					solver_threads[j] = thread(&Solver::solve_sop, solvers[j], first_visit);
				}
			}
			first_visits_mutex.unlock();
			for(int j = 0; j < thread_count; ++j){
				if(solver_threads[j].joinable()){
					solver_threads[j].join();
				}
			}
		}
	}
}

void Solver::split_visits(){
	int initial_size = first_visits.size();
	bool split = true;
	int index = 0;
	while(split && index < first_visits.size()){
		if(first_visits.empty() || first_visits.size() > thread_count){
			split = false;
		} else {
			SolverState first_visit = first_visits.top();;
			visited_nodes.assign(cost_graph->node_count(), false);
			for(const Edge& e: first_visit.path){
				visited_nodes[e.dest] = true;
			}
			if(first_visit.path.size() < cost_graph->node_count()-1){
				first_visits.pop();
				--index;
				first_visit.path.push_back(first_visit.next_edge);
				first_visit.cost += first_visit.next_edge.weight;
				visited_nodes[first_visit.next_edge.dest] = true;
				if(first_visit.cost < best_solution){
					for(const Edge& e : cost_graph->adj_outgoing(first_visit.next_edge.dest)){
						if(valid_node(e.dest)){
							first_visits.push(SolverState(e, first_visit.path, first_visit.cost));
						}
					}
				}
			}
		}
		++index;
	}
}

void Solver::solve_sop(SolverState first_visit){
	std::chrono::time_point<std::chrono::system_clock> current_time;
	int solution_size = cost_graph->node_count();
	bool start_new_search = true;
	while(start_new_search){
		vector<Edge> st;
		reset_solution(first_visit);
		st.push_back(first_visit.next_edge);
		while(!st.empty()){
			Edge last_edge = st.back();
			st.pop_back();
			solution.push_back(last_edge);
			solution_weight += last_edge.weight;
			visited_nodes[last_edge.dest] = true;
			last_visited_node = last_edge.dest;
			if(solution.size() == solution_size){
				update_best_solution();
				if(!st.empty()){
					backtrack(st.back().source);
				}
			}else if(better_history(solution_weight, last_edge.dest)){
				++counter;
				const vector<Edge>& sorted_outgoing = cost_graph->sorted_adj_outgoing(last_edge.dest);
				for(int i = sorted_outgoing.size() - 1; i >= 0; --i){
					if(valid_node(sorted_outgoing[i].dest)){
						st.push_back(sorted_outgoing[i]);
					}
				}
			} else {
				if(!st.empty()){
					backtrack(st.back().source);
				}
			}
			current_time = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_time = current_time - start_time;
			if(elapsed_time.count() > time_limit){
				break;
			}
		}
		first_visits_mutex.lock();
		if(!first_visits.empty()){
			if(first_visits.size() < thread_count){
				split_visits();
			}
			if(!first_visits.empty()){
				first_visit = first_visits.top();
				first_visits.pop();
			}
		} else {
			start_new_search = false;
		}
		first_visits_mutex.unlock();
	}
}

void Solver::backtrack(int source){
	while(solution.back().dest != source){
		Edge e = solution.back();
		solution.pop_back();
		visited_nodes[e.dest] = false;
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
	best_solution_mutex.lock();
	if(solution_weight < best_solution){
		best_solution = solution_weight;
		best_solution_nodes = solution;
	}
	best_solution_mutex.unlock();
}

void Solver::reset_solution(SolverState state){
	solution.clear();
	solution = state.path;
	solution_weight = state.cost;
	visited_nodes.assign(cost_graph->node_count(), false);
	for(const Edge& e: state.path){
		visited_nodes[e.dest] = true;
	}
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

bool Solver::better_history(int cost, int current_node){
	pair<std::vector<bool>, int> history_pair = pair<std::vector<bool>, int>(visited_nodes, last_visited_node);
	auto p = history.find(history_pair);
	bool continue_search = false;
	if(p.first != history.end()){
		if(p.first->second.prefix_cost > cost){
			history.lock_bucket(p.second);
			int improvement = p.first->second.prefix_cost - cost;
			
			if(p.first->second.lower_bound - improvement < best_solution){
				continue_search = true;
				Edge last_edge = solution.back();
				p.first->second.prefix_cost = cost;
				p.first->second.lower_bound = p.first->second.lower_bound - improvement;
			} else {
				if(p.first->second.prefix_cost > cost){
					p.first->second.prefix_cost = cost;
					p.first->second.lower_bound = p.first->second.lower_bound - improvement;
				}
			}
			history.unlock_bucket(p.second);
		}
	} else {
		int bound = edge_bound(current_node);
		
		p = history.find(history_pair);
		if(p.first == history.end()){
			history.put(history_pair, HistoryNode(cost, bound));
		} else {
			if(p.first->second.prefix_cost > cost){
				history.lock_bucket(p.second);
				int improvement = p.first->second.prefix_cost - cost;
				if(p.first->second.lower_bound - improvement < best_solution){
					continue_search = true;
					Edge last_edge = solution.back();
					p.first->second.prefix_cost = cost;
					p.first->second.lower_bound = p.first->second.lower_bound - improvement;
				} else {
					p.first->second.prefix_cost = cost;
					p.first->second.lower_bound = p.first->second.lower_bound - improvement;
				}
				history.unlock_bucket(p.second);
			}
		}
		
		continue_search = bound < best_solution;
	}
	return continue_search;
}

void Solver::nearest_neighbor(){
	Edge current_node(0,0,0);
	static_lower_bound = edge_bound(0);
	reset_solution(SolverState(current_node, vector<Edge>(), 0));
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

