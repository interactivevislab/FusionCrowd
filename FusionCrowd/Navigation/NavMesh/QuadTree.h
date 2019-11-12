#pragma once

#include <vector>
#include <memory>

#include "Math/Util.h"
#include "Math/BoundingBox.h"
#include "NavMesh.h"

namespace FusionCrowd
{
	class QuadTree
	{
	public:
		QuadTree(const NavMesh & navMesh);

		std::vector<size_t> GetContainingBBIds(DirectX::SimpleMath::Vector2 point);
	private:
		struct QuadTreeNode
		{
			bool LeafNode = false;
			float xmid, ymid;

			std::unique_ptr<QuadTreeNode> topLeft;
			std::unique_ptr<QuadTreeNode> topRight;
			std::unique_ptr<QuadTreeNode> bottomLeft;
			std::unique_ptr<QuadTreeNode> bottomRight;

			std::vector<size_t> meshNodeIds;
		};

		struct BoxWithId
		{
			BoxWithId(BoundingBox box, size_t id) : box(box), nodeId(id)
			{};

			BoundingBox box;
			size_t nodeId;
		};

		void BuildSubTree(QuadTreeNode& node, BoundingBox box, const NavMesh & navMesh, std::vector<BoxWithId> & ids, size_t level);

		const size_t _maxLevel = 5;
		std::unique_ptr<QuadTreeNode> _rootNode;
	};
}
