#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unordered_set>
#include <sys/types.h>
#include <dirent.h>

#include "digraph.h"
#include "solver.h"
#include "edge.h"

using std::string;
using std::ifstream;
using std::vector;
using std::cout;
using std::unordered_set;

void creat_graphs_from_file(string file, Digraph& g, Digraph& p);
void remove_redundant_edges(Digraph& g, Digraph& p);
void remove_redundant_edge_successors(Digraph& g, Digraph& p);
void print_solution_path(const vector<Edge>& path);

struct tour {
    vector<Edge> path;
    int cost;
};

struct tour read_tour_file(string file, const Digraph& g);
string match_file(const string& directory, string pattern);

int main(int argc, char *argv[]){
	if(argc != 4 && argc != 5 && argc != 6){
		std::cout << "Usage: ./main <file name> <time limit> <hash table size> [mode] [per_node]" << std::endl;
		return 1;
	}
	bool full_print = false;
	bool per_node_time_limit = false;
	string full("full");
	string per_node("per_node");
	if(argc == 5 && full.compare(argv[4]) == 0){
		full_print = true;
	} else if(argc == 5 && per_node.compare(argv[4]) == 0){
	    per_node_time_limit = true;
	} else if(argc == 6){
	    if(full.compare(argv[4]) == 0){
	        full_print = true;
	    }
	    if(per_node.compare(argv[5]) == 0){
	        per_node_time_limit = true;
	    }
	}
		
	Digraph g;
	Digraph p;
	creat_graphs_from_file(argv[1], g, p);
	g.sort_edges();
	remove_redundant_edges(g, p);
	remove_redundant_edge_successors(g, p);
	Solver s = Solver(&g, &p);
	s.set_time_limit(std::stoi(argv[2]), per_node_time_limit);
	s.set_hash_size(std::stoi(argv[3]));
	
	string file_name = argv[1];
    string delimiter = "/";
    size_t pos = file_name.find(delimiter);
    size_t pos_end = file_name.find(".sop");
    string folder = file_name.substr(0, pos);
    string problem = file_name.substr(pos+1, pos_end-pos);
    
    string path = "TOURS";
    if(folder == "tsplib"){
        path += "/TSPLIB";
    } else if(folder == "soplib"){
        path += "/SOPLIB";
    } else if(folder == "mibench"){
        path += "/COMPILERS";
    } else {
        path = "";
    }
    
    struct tour start_tour;
    if(path != ""){
        string matching_file = match_file(path, problem);
        if(matching_file != ""){
            start_tour = read_tour_file(path + "/" + matching_file, g);
            s.set_initial_solution(start_tour.path, start_tour.cost);
        }
    }
    s.nearest_neighbor();
    		
	int static_lower_bound = s.get_static_lower_bound();
	int nearest_neighbor_cost = s.best_solution_cost();
	if(full_print){
		cout << "static lower bound: " << static_lower_bound << std::endl;
		cout << "NN solution is" << std::endl;
		print_solution_path(s.best_solution_path());
		cout << "with cost: " << nearest_neighbor_cost << std::endl;
	}
	
	
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	s.solve_sop();
	
	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_time = end - start;
	int best_solution_cost = s.best_solution_cost();
	if(full_print){
		cout << "best solution found" << std::endl;
		print_solution_path(s.best_solution_path());
		cout << "with cost: " << best_solution_cost << std::endl;
		cout << "time duration (seconds): " << elapsed_time.count() << std::endl;
		std::cout << "enumerated nodes: " << s.get_enumerated_nodes() << std::endl;
        std::cout << "calculated bounds: " << s.get_bound_calculations() << std::endl;
	} else {
		cout << static_lower_bound << "," << nearest_neighbor_cost << "," << best_solution_cost << "," << elapsed_time.count();
		 cout << "," << s.get_enumerated_nodes() << "," << s.get_bound_calculations() << std::endl;
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
        unordered_set<int> expanded_nodes;
        for(int j = 0; j < preceding_nodes.size(); ++j){
            vector<Edge> st;
            st.push_back(preceding_nodes[j]);
            while(!st.empty()){
                Edge dependence_edge = st.back();
                st.pop_back();
                if(dependence_edge.source != i){
                    g.remove_edge(dependence_edge.dest, i);
                    expanded_nodes.insert(dependence_edge.dest);
                }
                for(const Edge& e : p.adj_outgoing(dependence_edge.dest)){
                    if(expanded_nodes.find(e.dest) == expanded_nodes.end()){
                        st.push_back(e);
                        expanded_nodes.insert(e.dest);
                    }
                }
            }
        } 
    }
}

void remove_redundant_edge_successors(Digraph& g, Digraph& p){
    for(int i = 0; i < p.node_count(); ++i){
        const vector<Edge>& preceding_nodes = p.adj_incoming(i);
        unordered_set<int> expanded_nodes;
        for(int j = 0; j < preceding_nodes.size(); ++j){
            vector<Edge> st;
            st.push_back(preceding_nodes[j]);
            while(!st.empty()){
                Edge dependence_edge = st.back();
                st.pop_back();
                if(dependence_edge.source != i){
                    g.remove_edge(i, dependence_edge.dest);
                    expanded_nodes.insert(dependence_edge.dest);
                }
                for(const Edge& e : p.adj_outgoing(dependence_edge.dest)){
                    if(expanded_nodes.find(e.dest) == expanded_nodes.end()){
                        st.push_back(e);
                        expanded_nodes.insert(e.dest);
                    }
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

struct tour read_tour_file(string file, const Digraph& g){
    vector<vector<int>> cost_matrix = g.dense_hungarian();
    struct tour t;
    t.path = vector<Edge>();
    t.cost = 0;
    ifstream tour_file(file);
	if(tour_file.fail()){
		std::cout << "failed to open file at " << file << std::endl;
		exit(1);
	}
	string line;
	int row = 0;
	int prev = 0;
	int next = 0;
	bool set_size = true;
	while(getline(tour_file, line)){
		if(row > 5 && line != "EOF"){
		    next = std::stoi(line);
		    next -= 1;
		    if(next >= 0){
		        int weight = cost_matrix[prev][next]/2;
		        if(next == prev){
		            weight = 0;
		        }
		        t.path.push_back(Edge(prev, next, weight));
		        t.cost += weight;
		    }
		    prev = next;
		}
		++row;
	}
	return t;
}

string match_file(const string& directory, string pattern){
    string file_match = "";
    DIR* dirp = opendir(directory.c_str());
    struct dirent * dp;
    if(dirp != NULL){
        while ((dp = readdir(dirp)) != NULL) {
            string file = string(dp->d_name);
            size_t found = file.find(pattern);
            if(found != string::npos){
                file_match = file;
            }
        }
    closedir(dirp);
    }
    
    return file_match;
}


