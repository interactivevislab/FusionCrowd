#include "QuadTree.h"

#include <set>
#include <deque>
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

		_rootNode = std::make_unique<Node>();

		BuildSubTree(*_rootNode.get(), BB, boxes, 0);
	}

	void QuadTree::BuildSubTree(Node& node, BoundingBox box, std::vector<Box> & boxes, size_t level)
	{
		float xmid = (box.xmin + box.xmax) / 2;
		float ymid = (box.ymin + box.ymax) / 2;
		node.LeafNode = (level == _maxLevel || boxes.size() <= 1);

		for(auto & bi : boxes)
		{
			node.meshNodeIds.push_back(bi.objectId);
		}

		if(node.LeafNode)
		{
			return;
		}

		BoundingBox tl { box.xmin, box.ymin,      xmid,     ymid };
		BoundingBox tr {     xmid, box.ymin,  box.xmax,     ymid };
		BoundingBox bl { box.xmin,     ymid,      xmid, box.ymax };
		BoundingBox br {     xmid,     ymid,  box.xmax, box.ymax };

		std::vector<Box> topLefts;
		std::vector<Box> topRights;
		std::vector<Box> bottomLefts;
		std::vector<Box> bottomRights;

		for(auto & bi : boxes)
		{
			if(tl.Overlaps(bi.bb))
			{
				topLefts.push_back(bi);
			}

			if(tr.Overlaps(bi.bb))
			{
				topRights.push_back(bi);
			}

			if(bl.Overlaps(bi.bb))
			{
				bottomLefts.push_back(bi);
			}

			if(br.Overlaps(bi.bb))
			{
				bottomRights.push_back(bi);
			}
		}

		node.topLeftBB = tl;
		node.topLeft = std::make_unique<Node>();

		node.topRightBB = tr;
		node.topRight = std::make_unique<Node>();

		node.bottomLeftBB = bl;
		node.bottomLeft = std::make_unique<Node>();

		node.bottomRightBB = br;
		node.bottomRight = std::make_unique<Node>();

		BuildSubTree(*node.topLeft.get(), tl, topLefts, level + 1);
		BuildSubTree(*node.topRight.get(), tr, topRights, level + 1);
		BuildSubTree(*node.bottomLeft.get(), bl, bottomLefts, level + 1);
		BuildSubTree(*node.bottomRight.get(), br, bottomRights, level + 1);
	}


	std::vector<size_t> QuadTree::GetContainingBBIds(DirectX::SimpleMath::Vector2 point)
	{
		Node * current = _rootNode.get();
		float x = point.x;
		float y = point.y;

		while(!current->LeafNode)
		{
			if(current->topLeftBB.Contains(x, y))
			{
				current = current->topLeft.get();
				continue;
			}

			if(current->topRightBB.Contains(x, y))
			{
				current = current->topRight.get();
				continue;
			}

			if(current->bottomLeftBB.Contains(x, y))
			{
				current = current->bottomLeft.get();
				continue;
			}

			if(current->bottomRightBB.Contains(x, y))
			{
				current = current->bottomRight.get();
				continue;
			}

			//outside of the boundaries
			return std::vector<size_t>{};
		}

		return current->meshNodeIds;
	}

	std::vector<size_t> QuadTree::GetIntersectingBBIds(BoundingBox box)
	{
		std::set<size_t> result;
		std::deque<Node*> processing;

		processing.push_back(_rootNode.get());

		while(processing.size() > 0)
		{
			Node * current = processing.front(); processing.pop_front();

			if(current->LeafNode)
			{
				result.insert(current->meshNodeIds.begin(), current->meshNodeIds.end());
				continue;
			}

			if(box.Overlaps(current->topLeftBB))
			{
				processing.push_back(current->topLeft.get());
			}

			if(box.Overlaps(current->topRightBB))
			{
				processing.push_back(current->topRight.get());
			}

			if(box.Overlaps(current->bottomLeftBB))
			{
				processing.push_back(current->bottomLeft.get());
			}

			if(box.Overlaps(current->bottomRightBB))
			{
				processing.push_back(current->bottomRight.get());
			}
		}

		return std::vector<size_t>(result.begin(), result.end());
	}
}