#include "edge.h"

Edge::Edge(int source, int dest, int weight){
	this->source = source;
	this->dest = dest;
	this->weight = weight;
}

bool operator<(const Edge& first, const Edge& second){
	return first.weight < second.weight;
}
