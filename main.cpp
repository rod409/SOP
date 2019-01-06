#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <chrono>

#include "digraph.h"
#include "solver.h"
#include "edge.h"

using std::string;
using std::ifstream;
using std::vector;
using std::cout;

void creat_graphs_from_file(string file, Digraph& g, Digraph& p);
void remove_redundant_edges(Digraph& g, Digraph& p);
void print_solution_path(const vector<Edge>& path);

int main(int argc, char *argv[]){
	if(argc != 5 && argc != 6){
		std::cout << "Usage: ./main <file name> <time limit> <hash table size> [mode] <number of threads>" << std::endl;
		return 1;
	}
	bool full_print = false;
	string full("full");
	if(argc == 6 && full.compare(argv[4]) == 0){
		full_print = true;
	}
	
	Digraph g;
	Digraph p;
	creat_graphs_from_file(argv[1], g, p);
	remove_redundant_edges(g, p);
	g.sort_edges();
	Solver s = Solver(&g, &p);
	s.set_time_limit_per_node(std::stoi(argv[2]));
	s.set_hash_size(std::stoi(argv[3]));
	
	s.nearest_neighbor();
	int static_lower_bound = s.get_static_lower_bound();
	int nearest_neighbor_cost = s.best_solution_cost();
	int num_threads;
	if(full_print){
		cout << "static lower bound: " << static_lower_bound << std::endl;
		cout << "NN solution is" << std::endl;
		print_solution_path(s.best_solution_path());
		cout << "with cost: " << nearest_neighbor_cost << std::endl;
		num_threads = std::stoi(argv[5]);
	} else {
		num_threads = std::stoi(argv[4]);
	}
	
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	s.solve_sop_parallel(num_threads);
	
	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_time = end - start;
	int best_solution_cost = s.best_solution_cost();
	if(full_print){
		cout << "best solution found" << std::endl;
		print_solution_path(s.best_solution_path());
		cout << "with cost: " << s.best_solution_cost() << std::endl;
		std::cout << "time duration (seconds): " << elapsed_time.count() << std::endl;
	} else {
		cout << static_lower_bound << "," << nearest_neighbor_cost << "," << best_solution_cost << "," << elapsed_time.count() << std::endl;
	}
	
	return 0;
}

void creat_graphs_from_file(string file, Digraph& g, Digraph& p){
	ifstream graph_file(file);
	if(graph_file.fail()){
		std::cout << "failed to open file at " << file << std::endl;
		exit(1);
	}
	string line;
	int source = 0;
	bool set_size = true;
	while(getline(graph_file, line)){
		std::istringstream iss(line);
		vector<string> words;
		for(std::string s; iss >> s; ){
    		words.push_back(s);
		}
		if(set_size){
			g.set_size(words.size());
			p.set_size(words.size());
			set_size = false;
		}
		for(int i = 0; i < words.size(); ++i){
			int dest = i;
			int weight = std::stoi(words[i]);
			if(weight < 0){
				p.add_edge(source, dest, 0);
			} else if(source != dest) {
				g.add_edge(source, dest, weight);
			}
		}
		++source;
	}
}

void remove_redundant_edges(Digraph& g, Digraph& p){
    for(int i = 0; i < p.node_count(); ++i){
        const vector<Edge>& preceding_nodes = p.adj_outgoing(i);
        for(int j = 0; j < preceding_nodes.size(); ++j){
            vector<Edge> st;
            st.push_back(preceding_nodes[j]);
            while(!st.empty()){
                Edge dependence_edge = st.back();
                st.pop_back();
                if(dependence_edge.source != i){
                    g.remove_edge(dependence_edge.dest, i);
                }
                for(const Edge& e : p.adj_outgoing(dependence_edge.dest)){
                    st.push_back(e);
                }
            }
        }
        
        
    }
}

void print_solution_path(const vector<Edge>& path){
	for(Edge e : path){
		cout << e.dest << " -> ";
	}
	cout << std::endl;
}


