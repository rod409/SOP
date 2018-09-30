#ifndef HASH_MAP_H
#define HASH_MAP_H

#include <vector>
#include <list>
#include <utility>
#include <cstddef>
#include <string>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/sysinfo.h>

using std::vector;
using std::list;
using std::pair;
using std::size_t;


template <typename K, typename V>
class HashMap {
	public:
		HashMap();
		void set_size(size_t size, size_t bit_vector_mem_size);
		pair<K, V> * find(const K& k);
		void put(const K& k, V v);
		pair<K, V> const * end();
	private:
		vector<list<pair<K, V>>> table;
		pair<K, V> end_entry;
		size_t size;
		size_t max_size;
		size_t current_size;
		size_t element_size;
};

template <typename K, typename V>
HashMap<K, V>::HashMap(){
	size = 49157;
	table.resize(size);
}

template <typename K, typename V>
void HashMap<K, V>::set_size(size_t size, size_t bit_vector_mem_size){
	table.resize(size);
	this->size = size;
	
	struct sysinfo info;
	if(sysinfo(&info) != 0){
		std::cerr << std::strerror(errno) << std::endl;
		std::exit(1);
	}
	
	max_size = (double)info.freeram*0.75;
	element_size = 2*sizeof(V*)+sizeof(V) + bit_vector_mem_size;
	current_size = 0;
}

template <typename K, typename V>
pair<K, V> * HashMap<K, V>::find(const K& k){
	size_t hash_bucket = (std::hash<K>()(k))%size;
	for(typename list<pair<K, V>>::iterator it = table[hash_bucket].begin(); it != table[hash_bucket].end(); it++){
		if(it->first == k){
			return &*it;
		}
	}
	return &end_entry;
}

template <typename K, typename V>
void HashMap<K, V>::put(const K& k, V v){
	size_t hash_bucket = (std::hash<K>()(k))%size;
	for(pair<K, V> p: table[hash_bucket]){
		if(p.first == k){
			p.second = v;
			return;
		}
	}
	if(current_size < max_size){
		table[hash_bucket].push_back(pair<K, V>(k, v));
		current_size += element_size;
	}
}

template <typename K, typename V>
pair<K, V> const * HashMap<K, V>::end(){
	return &end_entry;
}

#endif
