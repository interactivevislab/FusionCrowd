#include "QuadTree.h"

#include <queue>
#include <algorithm>

namespace FusionCrowd
{
	QuadTree::QuadTree(std::vector<Box> boxes, size_t maxLevels) : _maxLevel(maxLevels)
	{
		BoundingBox BB {0, 0, 0, 0};

		for(auto & box : boxes)
		{
			BB = BB.Union(box.bb);
		}

		_bb = BB;
		_rootNode = std::make_unique<Node>();

		BuildSubTree(*_rootNode.get(), BB, boxes, 0);
	}

	void QuadTree::BuildSubTree(Node& node, BoundingBox box, std::vector<Box> & boxes, size_t level)
	{
		node.xmid = (box.xmin + box.xmax) / 2;
		node.ymid = (box.ymin + box.ymax) / 2;
		node.LeafNode = (level == _maxLevel || boxes.size() <= 1);

		if(node.LeafNode)
		{
			for(auto & bi : boxes)
			{
				node.meshNodeIds.push_back(bi.objectId);
			}

			return;
		}

		BoundingBox tl { box.xmin, box.ymin, node.xmid, node.ymid };
		BoundingBox tr {node.xmid, box.ymin,  box.xmax, node.ymid };
		BoundingBox bl { box.xmin,node.ymid, node.xmid,  box.ymax };
		BoundingBox br {node.xmid,node.ymid,  box.xmax,  box.ymax };

		std::vector<Box> topLefts;
		std::vector<Box> topRights;
		std::vector<Box> bottomLefts;
		std::vector<Box> bottomRights;

		for(auto & bi : boxes)
		{
			if(bi.bb.xmax <= node.xmid && bi.bb.ymax <= node.ymid)
			{
				topLefts.push_back(bi);
				continue;
			}

			if(bi.bb.xmin >= node.xmid && bi.bb.ymax <= node.ymid)
			{
				topRights.push_back(bi);
				continue;
			}

			if(bi.bb.xmax <= node.xmid && bi.bb.ymin >= node.ymid)
			{
				bottomLefts.push_back(bi);
				continue;
			}

			if(bi.bb.xmin >= node.xmid && bi.bb.ymin >= node.ymid)
			{
				bottomRights.push_back(bi);
				continue;
			}

			node.meshNodeIds.push_back(bi.objectId);
		}

		node.topLeft = std::make_unique<Node>();
		node.topRight = std::make_unique<Node>();
		node.bottomLeft = std::make_unique<Node>();
		node.bottomRight = std::make_unique<Node>();

		BuildSubTree(*node.topLeft.get(), tl, topLefts, level + 1);
		BuildSubTree(*node.topRight.get(), tr, topRights, level + 1);
		BuildSubTree(*node.bottomLeft.get(), bl, bottomLefts, level + 1);
		BuildSubTree(*node.bottomRight.get(), br, bottomRights, level + 1);
	}


	std::vector<size_t> QuadTree::GetContainingBBIds(DirectX::SimpleMath::Vector2 point)
	{
		std::vector<size_t> result;
		Node * current = _rootNode.get();
		float x = point.x;
		float y = point.y;

		while(true)
		{
			result.insert(std::end(result), std::begin(current->meshNodeIds), std::end(current->meshNodeIds));

			if(current->LeafNode)
				break;

			if(x <= current->xmid && y <= current->ymid)
			{
				current = current->topLeft.get();
				continue;
			}

			if(x >= current->xmid && y <= current->ymid)
			{
				current = current->topRight.get();
				continue;
			}

			if(x <= current->xmid && y >= current->ymid)
			{
				current = current->bottomLeft.get();
				continue;
			}

			if(x >= current->xmid && y >= current->ymid)
			{
				current = current->bottomRight.get();
				continue;
			}

			//outside of the boundaries
			break;
		};

		return result;
	}

	std::vector<size_t> QuadTree::GetIntersectingBBIds(BoundingBox box)
	{
		std::vector<size_t> result;
		std::queue<Node *> dq;

		dq.push(_rootNode.get());

		while(dq.size() > 0)
		{
			Node * current = dq.front(); dq.pop();

			result.insert(std::end(result), std::begin(current->meshNodeIds), std::end(current->meshNodeIds));

			if(current->LeafNode)
			{
				continue;
			}

			if(box.xmin <= current->xmid && box.ymin <= current->ymid)
			{
				dq.push(current->topLeft.get());
			}

			if(box.xmax >= current->xmid && box.ymin <= current->ymid)
			{
				dq.push(current->topRight.get());
			}

			if(box.xmin <= current->xmid && box.ymax >= current->ymid)
			{
				dq.push(current->bottomLeft.get());
			}

			if(box.xmax >= current->xmid && box.ymax >= current->ymid)
			{
				dq.push(current->bottomRight.get());
			}
		}

		return result;
	}
}