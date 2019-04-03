#pragma once

class AStarMinHeap
{
public:
	AStarMinHeap(unsigned int * heap, float * data, bool * state, unsigned int * path,
		size_t N);
	bool empty() const { return _nextFree == 0; }
	unsigned int pop();
	void push(unsigned int x);
	inline void g(unsigned int node, float value) { _g[node] = value; }
	inline float g(unsigned int node) const { return _g[node]; }
	inline void h(unsigned int node, float value) { _h[node] = value; }
	inline float h(unsigned int node) const { return _h[node]; }
	inline void f(unsigned int node, float value)
	{
		if (_inHeap[node]) changeF(node, value);
		else _f[node] = value;
	}
	inline float f(unsigned int node) const { return _f[node]; }
	void changeF(unsigned int node, float value);
	inline bool isVisited(unsigned int node) const { return _visited[node]; }
	inline bool isInHeap(unsigned int node) const { return _inHeap[node]; }
	inline void setReachedFrom(unsigned int dst, unsigned int src)
	{
		_cameFrom[dst] = src;
	}
	inline unsigned int getReachedFrom(unsigned int dst) const {
		return _cameFrom[dst];
	}
protected:
	void initialize(size_t N);
	float	_minKey;
	unsigned int _minIdx;
	unsigned int _nextFree;
	float *	_f;
	float * _g;
	float * _h;
	bool  * _inHeap;
	bool  * _visited;
	unsigned int * _heap;
	unsigned int * _cameFrom;
};
