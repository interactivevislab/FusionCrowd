#pragma once

#include <vector>
#include <set>
#include <memory>
#include <queue>

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
		void UpdateTree(std::vector<Box>& add_boxes, std::vector<size_t>& del_boxes);
	private:
		struct Node
		{
			Node(size_t start) : start(start), len(0) {}

			float xmid = 0.0, ymid = 0.0;

			size_t start = 0;
			size_t len   = 0;

			size_t firstChild = -1;

			inline bool LeafNode() const { return firstChild == -1; };

			inline size_t topLeft() const     { return firstChild + 0; };
			inline size_t topRight() const    { return firstChild + 1; };
			inline size_t bottomLeft() const  { return firstChild + 2; };
			inline size_t bottomRight() const { return firstChild + 3; };
		};

		std::vector<Box> _stored_boxes;
		std::vector<Node> _stored_nodes;

		size_t SmallestContainingNode(BoundingBox & box);
		size_t BuildSubTree(size_t node, BoundingBox box, std::vector<Box> & boxes, size_t level);

		BoundingBox _bb;
		const size_t _maxLevel;
		const size_t _rootNode = 0;

		std::queue<size_t> _query_queue;

		float _query_avg_weight = 0.5f;
		size_t _containing_query_result_moving_avg = 0;
		size_t _intersecting_query_result_moving_avg = 0;

		void MakeDepthWalk(std::vector<size_t>& depth_walk, size_t parent_pos, std::set<size_t>& visited);
	};
}
