#include "QuadTree.h"

#include <deque>
#include <algorithm>

namespace FusionCrowd
{
	QuadTree::QuadTree(const NavMesh & navMesh)
	{
		BoundingBox box {1e7f, 1e7f, -1e7f, -1e7f };

		const size_t NODE_COUNT = navMesh.getNodeCount();

		std::vector<BoxWithId> ids;
		for(size_t nodeId = 0; nodeId < NODE_COUNT; nodeId ++)
		{
			auto & bb = navMesh.GetNode(nodeId).GetBB();
			box = box.Union(bb);
			ids.push_back({ bb, nodeId });
		}

		_rootNode = std::make_unique<QuadTreeNode>();

		BuildSubTree(*_rootNode.get(), box, navMesh, ids, 0);
	}

	void QuadTree::BuildSubTree(QuadTreeNode& node, BoundingBox box, const NavMesh & navMesh, std::vector<BoxWithId> & ids, size_t level)
	{
		float xmid = (box.xmin + box.xmax) / 2;
		float ymid = (box.ymin + box.ymax) / 2;
		node.xmid = xmid;
		node.ymid = ymid;
		node.LeafNode = level == _maxLevel;

		for(auto & bi : ids)
		{
			node.meshNodeIds.push_back(bi.nodeId);
		}

		if(node.LeafNode)
		{
			return;
		}

		BoundingBox tl { box.xmin, box.ymin,      xmid,     ymid };
		BoundingBox tr {     xmid, box.ymin,  box.xmax,     ymid };
		BoundingBox bl { box.xmin,     ymid,      xmid, box.ymax };
		BoundingBox br {     xmid,     ymid,  box.xmax, box.ymax };

		std::vector<BoxWithId> topLefts;
		std::vector<BoxWithId> topRights;
		std::vector<BoxWithId> bottomLefts;
		std::vector<BoxWithId> bottomRights;

		for(auto & bi : ids)
		{
			if(tl.Overlaps(bi.box))
			{
				topLefts.push_back(bi);
			}

			if(tr.Overlaps(bi.box))
			{
				topRights.push_back(bi);
			}

			if(bl.Overlaps(bi.box))
			{
				bottomLefts.push_back(bi);
			}

			if(br.Overlaps(bi.box))
			{
				bottomRights.push_back(bi);
			}
		}

		node.topLeft = std::make_unique<QuadTreeNode>();
		node.topRight = std::make_unique<QuadTreeNode>();
		node.bottomLeft = std::make_unique<QuadTreeNode>();
		node.bottomRight = std::make_unique<QuadTreeNode>();

		BuildSubTree(*node.topLeft.get(), tl, navMesh, topLefts, level + 1);
		BuildSubTree(*node.topRight.get(), tr, navMesh, topRights, level + 1);
		BuildSubTree(*node.bottomLeft.get(), bl, navMesh, bottomLefts, level + 1);
		BuildSubTree(*node.bottomRight.get(), br, navMesh, bottomRights, level + 1);
	}


	std::vector<size_t> QuadTree::GetContainingBBIds(DirectX::SimpleMath::Vector2 point)
	{
		QuadTreeNode * current = _rootNode.get();

		while(!current->LeafNode)
		{
			if(point.x < current->xmid)
			{
				if(point.y < current->ymid)
				{
					current = current->topLeft.get();
				} else
				{
					current = current->bottomLeft.get();
				}
			} else
			{
				if(point.y < current->ymid)
				{
					current = current->topRight.get();
				} else
				{
					current = current->bottomRight.get();
				}
			}
		}

		return current->meshNodeIds;
	}
}