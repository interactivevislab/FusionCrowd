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
		_rootNode = std::make_unique<Node>(0);
		_stored_boxes.resize(boxes.size());

		BuildSubTree(*_rootNode.get(), BB, boxes, 0);
	}

	size_t QuadTree::BuildSubTree(Node& node, BoundingBox box, std::vector<Box> & boxes, size_t level)
	{
		node.LeafNode = (level == _maxLevel || boxes.size() <= 1);

		if(node.LeafNode)
		{
			node.xmid = (box.xmin + box.xmax) / 2.0;
			node.ymid = (box.ymin + box.ymax) / 2.0;
			node.len = boxes.size();

			for(size_t t = 0; t < boxes.size(); t++)
			{
				_stored_boxes[node.start + t] = boxes[t];
			}

			return node.start + node.len;
		}

		auto boxCount = boxes.size();
		std::vector<float> xs(boxCount * 2);
		std::vector<float> ys(boxCount * 2);

		for(auto & bi : boxes)
		{
			xs.push_back(bi.bb.xmin);
			xs.push_back(bi.bb.xmax);
			ys.push_back(bi.bb.ymin);
			ys.push_back(bi.bb.ymax);
		}

		std::sort(std::begin(xs), std::end(xs));
		std::sort(std::begin(ys), std::end(ys));

		node.xmid = (xs[boxCount] + xs[boxCount + 1]) / 2;
		node.ymid = (ys[boxCount] + ys[boxCount + 1]) / 2;


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

			_stored_boxes[node.start + node.len] = bi;
			node.len++;
		}

		size_t newStart = node.start + node.len;

		node.topLeft = std::make_unique<Node>(newStart);
		newStart = BuildSubTree(*node.topLeft.get(), tl, topLefts, level + 1);

		node.topRight = std::make_unique<Node>(newStart);
		newStart = BuildSubTree(*node.topRight.get(), tr, topRights, level + 1);

		node.bottomLeft = std::make_unique<Node>(newStart);
		newStart = BuildSubTree(*node.bottomLeft.get(), bl, bottomLefts, level + 1);

		node.bottomRight = std::make_unique<Node>(newStart);
		return BuildSubTree(*node.bottomRight.get(), br, bottomRights, level + 1);
	}


	std::vector<size_t> QuadTree::GetContainingBBIds(DirectX::SimpleMath::Vector2 point)
	{
		std::vector<size_t> result;
		Node * current = _rootNode.get();
		float x = point.x;
		float y = point.y;

		while(true)
		{
			for(size_t idx = current->start; idx < current->start + current->len; idx++)
			{
				if(_stored_boxes[idx].bb.Contains(point.x, point.y))
					result.push_back(_stored_boxes[idx].objectId);
			}

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

			for(size_t idx = current->start; idx < current->start + current->len; idx++)
			{
				if(_stored_boxes[idx].bb.Overlaps(box))
					result.push_back(_stored_boxes[idx].objectId);
			}

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