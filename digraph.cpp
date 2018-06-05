#include <vector>
#include <algorithm>

#include "digraph.h"
#include "edge.h"

using std::vector;

Digraph::Digraph(){
	num_nodes = 0;
}

Digraph::Digraph(int node_count){
	outgoing_edges.resize(node_count);
	incoming_edges.resize(node_count);
	sorted_outgoing_edges.resize(node_count);
	sorted_incoming_edges.resize(node_count);
	num_nodes = node_count;
}

void Digraph::set_size(int size){
	outgoing_edges.resize(size);
	incoming_edges.resize(size);
	sorted_outgoing_edges.resize(size);
	sorted_incoming_edges.resize(size);
	num_nodes = size;
}


void Digraph::add_edge(int source, int dest, int weight){
	Edge out(source, dest, weight);
	outgoing_edges[source].push_back(out);
	incoming_edges[dest].push_back(out);
	sorted_outgoing_edges[source].push_back(out);
	sorted_incoming_edges[dest].push_back(out);
}

int Digraph::node_count() const{
	return num_nodes;
}

const vector<Edge>& Digraph::adj_outgoing(int node) const{
	return outgoing_edges[node];
}

const vector<Edge>& Digraph::adj_incoming(int node) const{
	return incoming_edges[node];
}

void Digraph::sort_edges(){
	for(int i = 0; i < num_nodes; ++i){
		std::sort(sorted_outgoing_edges[i].begin(), sorted_outgoing_edges[i].end());
		std::sort(sorted_incoming_edges[i].begin(), sorted_incoming_edges[i].end());
	}
	
}
const vector<Edge>& Digraph::sorted_adj_outgoing(int node) const{
	return sorted_outgoing_edges[node];
}

const vector<Edge>& Digraph::sorted_adj_incoming(int node) const{
	return sorted_incoming_edges[node];
}

