#pragma once

#include <fstream>

#include "NavMeshNode.h"
#include "Config.h"
#include "Navigation/Obstacle.h"
#include "Math/Util.h"

// FORWARD DECLARATIONS
class NavMeshNode;
class NavMesh;

class FUSION_CROWD_API NavMeshObstacle : public Obstacle
{
public:
	static size_t NO_NEIGHBOR_OBST;

	NavMeshObstacle() : Obstacle(), _node(0x0) {}

	bool LoadFromAscii(std::ifstream & f, DirectX::SimpleMath::Vector2 * vertices);
	inline const NavMeshNode * getNode() const { return _node; }

	~NavMeshObstacle();

	friend class NavMeshNode;
	friend class NavMesh;
protected:
	NavMeshNode *	_node;
};

