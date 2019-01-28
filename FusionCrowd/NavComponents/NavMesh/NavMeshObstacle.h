#pragma once

#include <fstream>

#include "NavMeshNode.h"
#include "../../Math/vector.h"
#include "../Obstacle.h"

// FORWARD DECLARATIONS
class NavMeshNode;
class NavMesh;

class NavMeshObstacle : public Obstacle
{
public:
	static size_t NO_NEIGHBOR_OBST;

	NavMeshObstacle() : Obstacle(), _node(0x0) {}

	bool LoadFromAscii(std::ifstream & f, FusionCrowd::Math::Vector2 * vertices);
	inline const NavMeshNode * getNode() const { return _node; }

	~NavMeshObstacle();

	friend class NavMeshNode;
	friend class NavMesh;
protected:
	NavMeshNode *	_node;
};

