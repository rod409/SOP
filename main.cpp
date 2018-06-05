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
void print_solution_path(const vector<Edge>& path);

int main(int argc, char *argv[]){
	if(argc != 3){
		std::cout << "Usage: test <file name> <time limit>" << std::endl;
		return 1;
	}
		
	Digraph g;
	Digraph p;
	creat_graphs_from_file(argv[1], g, p);
	g.sort_edges();
	Solver s = Solver(g, p);
	s.set_time_limit_per_node(std::stoi(argv[2]));
	
	s.nearest_neighbor();
	cout << "static lower bound: " << s.get_static_lower_bound() << std::endl;
	cout << "NN solution is" << std::endl;
	print_solution_path(s.best_solution_path());
	cout << "with cost: " << s.best_solution_cost() << std::endl;
	
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	s.solve_sop();
	
	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_time = end - start;
	
	cout << "best solution found" << std::endl;
	print_solution_path(s.best_solution_path());
	cout << "with cost: " << s.best_solution_cost() << std::endl;
	
	std::cout << "time duration (seconds): " << elapsed_time.count() << std::endl;
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

void print_solution_path(const vector<Edge>& path){
	for(Edge e : path){
		cout << e.dest << " -> ";
	}
	cout << std::endl;
}


