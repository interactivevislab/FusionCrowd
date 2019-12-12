#pragma once

#include <vector>
#include <unordered_set>
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
			Box(): bb(BoundingBox(0,0,0,0)), objectId(0) { }
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
			Node(size_t start) : start(start), len(0) {}

			bool LeafNode = false;

			float xmid, ymid;

			size_t start;
			size_t len;

			std::unique_ptr<Node> topLeft;
			std::unique_ptr<Node> topRight;
			std::unique_ptr<Node> bottomLeft;
			std::unique_ptr<Node> bottomRight;
		};

		std::vector<Box> _stored_boxes;

		size_t BuildSubTree(Node& node, BoundingBox box, std::vector<Box> & boxes, size_t level);

		BoundingBox _bb;
		const size_t _maxLevel;
		std::unique_ptr<Node> _rootNode;
	};
}
