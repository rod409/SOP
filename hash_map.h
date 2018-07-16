#ifndef HASH_MAP_H
#define HASH_MAP_H

#include <vector>
#include <list>
#include <utility>
#include <cstddef>
#include <mutex>

using std::vector;
using std::list;
using std::pair;
using std::size_t;
using std::mutex;

template <typename K, typename V>
class HashMap {
	public:
		HashMap();
		void set_size(size_t size);
		pair<pair<K, V> *, size_t> find(const K& k);
		void put(const K& k, V v);
		pair<K, V> const * end();
		void lock_bucket(size_t bucket);
		void unlock_bucket(size_t bucket);
	private:
		vector<list<pair<K, V>>> table;
		pair<K, V> end_entry;
		size_t size;
		vector<mutex> chain_mutex;
};

template <typename K, typename V>
HashMap<K, V>::HashMap(){
	size = 0;
}

template <typename K, typename V>
void HashMap<K, V>::set_size(size_t size){
	table.resize(size);
	chain_mutex = vector<mutex>(size);
	this->size = size;
}

template <typename K, typename V>
pair<pair<K, V> *, size_t> HashMap<K, V>::find(const K& k){
	size_t hash_bucket = (std::hash<K>()(k))%size;
	for(typename list<pair<K, V>>::iterator it = table[hash_bucket].begin(); it != table[hash_bucket].end(); it++){
		if(it->first == k){
			return pair<pair<K, V> *, size_t>(&*it, hash_bucket);
		}
	}
	return pair<pair<K, V> *, size_t>(&end_entry, hash_bucket);
}

template <typename K, typename V>
void HashMap<K, V>::put(const K& k, V v){
	size_t hash_bucket = (std::hash<K>()(k))%size;
	chain_mutex[hash_bucket].lock();
	for(pair<K, V> p: table[hash_bucket]){
		if(p.first == k){
			p.second = v;
			chain_mutex[hash_bucket].unlock();
			return;
		}
	}
	table[hash_bucket].push_back(pair<K, V>(k, v));
	chain_mutex[hash_bucket].unlock();
}

template <typename K, typename V>
pair<K, V> const * HashMap<K, V>::end(){
	return &end_entry;
}

template <typename K, typename V>
void HashMap<K, V>::lock_bucket(size_t bucket){
	chain_mutex[bucket].lock();
}

template <typename K, typename V>
void HashMap<K, V>::unlock_bucket(size_t bucket){
	chain_mutex[bucket].unlock();
}

#endif
