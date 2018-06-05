#ifndef DIGPRAPH_H
#define DIGPRAPH_H

#include <vector>
#include "edge.h"

class Digraph {
	public:
		Digraph();
		Digraph(int node_count);
		void set_size(int size);
		void add_edge (int source, int target, int weight);
		int node_count() const;
		const std::vector<Edge>& adj_outgoing(int node) const;
		const std::vector<Edge>& adj_incoming(int node) const;
		const std::vector<Edge>& sorted_adj_outgoing(int node) const;
		const std::vector<Edge>& sorted_adj_incoming(int node) const;
		void sort_edges();
	private:
		std::vector<std::vector<Edge>> outgoing_edges;
		std::vector<std::vector<Edge>> incoming_edges;
		std::vector<std::vector<Edge>> sorted_outgoing_edges;
		std::vector<std::vector<Edge>> sorted_incoming_edges;
		int num_nodes;
};

#endif
