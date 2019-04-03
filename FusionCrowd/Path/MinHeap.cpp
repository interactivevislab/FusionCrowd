#include "MinHeap.h"
#include <iostream>

AStarMinHeap::AStarMinHeap(unsigned int * heap, float * data, bool * state,
	unsigned int * path, size_t N) {
	_f = data;
	_g = _f + N;
	_h = _g + N;
	_inHeap = state;
	_visited = _inHeap + N;
	_heap = heap;
	_cameFrom = path;
	initialize(N);
}

void AStarMinHeap::initialize(size_t N) {
	// This code assumes that the f-value is the first block of the data
	//	and _inHeap is the first block of the state
	// TODO: Is it stritly necessary to initialize the values as a block?
	//		Can't they be implictly initialized by the value of _minIndex
	memset(_f, 0x7f, sizeof(float) * 3 * N);
	memset(_inHeap, 0x0, sizeof(bool) * 2 * N);
	// note: don't need to initialize the heap array
	_minIdx = _nextFree = 0;
	_minKey = 0.f;
}

unsigned int AStarMinHeap::pop() {
	unsigned int returnVal = _heap[_minIdx];
	_inHeap[returnVal] = false;
	_visited[returnVal] = true;
	// Swap into this slot as necessary
	--_nextFree;
	if (_minIdx != _nextFree) {
		_heap[_minIdx] = _heap[_nextFree];
	}
	// Identify the new minimum
	_minIdx = 0;
	_minKey = _f[_heap[0]];
	for (unsigned int i = 1; i < _nextFree; ++i) {
		const unsigned int x = _heap[i];
		if (_f[x] < _minKey) {
			_minIdx = i;
			_minKey = _f[x];
		}
	}
	return returnVal;
}

void AStarMinHeap::push(unsigned int x) {
	if (_f[x] < _minKey) {
		_minIdx = _nextFree;
		_minKey = _f[x];
	}
	_inHeap[x] = true;
	_heap[_nextFree] = x;
	++_nextFree;
}

void AStarMinHeap::changeF(unsigned int val, float key) {
	// Only has an impact if this introduces a new minimum
	if (key < _minKey) {
		_minKey = key;
		for (unsigned int i = 0; i < _nextFree; ++i) {
			if (_heap[i] == val) {
				_minIdx = i;
				break;
			}
		}
	}
	_f[val] = key;
}