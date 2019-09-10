#pragma once

#include <istream>

#include "Navigation/Obstacle.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	// FORWARD DECLARATIONS
	class NavMeshNode;
	class NavMesh;

	class NavMeshObstacle : public Obstacle
	{
	public:
		static size_t NO_NEIGHBOR_OBST;

		NavMeshObstacle() : Obstacle(), _node(0x0)
		{
		}

		bool LoadFromAscii(std::istream& f, DirectX::SimpleMath::Vector2* vertices);
		inline const NavMeshNode* getNode() const { return _node; }

		~NavMeshObstacle();

		friend class NavMeshNode;
		friend class NavMesh;
	protected:
		NavMeshNode* _node;
	};
}
