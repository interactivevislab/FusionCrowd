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
		struct Box
		{
			Box(BoundingBox bb, size_t objectId) : bb(bb), objectId(objectId) { };

			BoundingBox bb;
			size_t objectId;
		};

		QuadTree(std::vector<Box> boxes, size_t maxLevels = 5);

	public:
		std::vector<size_t> GetContainingBBIds(DirectX::SimpleMath::Vector2 point);
		std::vector<size_t> GetIntersectingBBIds(BoundingBox box);
	private:
		struct Node
		{
			bool LeafNode = false;

			BoundingBox topLeftBB;
			BoundingBox topRightBB;
			BoundingBox bottomLeftBB;
			BoundingBox bottomRightBB;

			std::unique_ptr<Node> topLeft;
			std::unique_ptr<Node> topRight;
			std::unique_ptr<Node> bottomLeft;
			std::unique_ptr<Node> bottomRight;

			std::vector<size_t> meshNodeIds;
		};

		void BuildSubTree(Node& node, BoundingBox box, std::vector<Box> & boxes, size_t level);

		const size_t _maxLevel;
		std::unique_ptr<Node> _rootNode;
	};
}
