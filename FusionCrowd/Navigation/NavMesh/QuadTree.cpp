#include "QuadTree.h"

#include <algorithm>
#include <map>

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
		_stored_nodes.push_back(Node(0));

		BuildSubTree(_rootNode, BB, boxes, 0);
	}

	size_t QuadTree::BuildSubTree(size_t nodeIdx, BoundingBox box, std::vector<Box> & boxes, size_t level)
	{
		Node & node = _stored_nodes[nodeIdx];

		bool leafNode = (level == _maxLevel || boxes.size() <= 1);
		node.len = 0;
		node.xmid = (box.xmin + box.xmax) / 2.0f;
		node.ymid = (box.ymin + box.ymax) / 2.0f;

		if(leafNode)
		{
			node.len = boxes.size();
			_stored_boxes.insert(_stored_boxes.end(), boxes.begin(), boxes.end());

			return node.start + node.len;
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

			_stored_boxes.push_back(bi);
			node.len++;
		}

		auto topLeft     = _stored_nodes.size();
		auto topRight    = topLeft + 1;
		auto bottomLeft  = topLeft + 2;
		auto bottomRight = topLeft + 3;

		node.firstChild = topLeft;

		size_t startNxt = node.start + node.len;

		//	We now start index only for the first one, for others it will be assigned later
		_stored_nodes.push_back(Node(startNxt));
		_stored_nodes.push_back(Node(0));
		_stored_nodes.push_back(Node(0));
		_stored_nodes.push_back(Node(0));

		_stored_nodes[   topRight] = Node(BuildSubTree(topLeft, tl, topLefts, level + 1));
		_stored_nodes[ bottomLeft] = Node(BuildSubTree(topRight, tr, topRights, level + 1));
		_stored_nodes[bottomRight] = Node(BuildSubTree(bottomLeft, bl, bottomLefts, level + 1));

		return BuildSubTree(bottomRight, br, bottomRights, level + 1);
	}


	std::vector<size_t> QuadTree::GetContainingBBIds(DirectX::SimpleMath::Vector2 point)
	{
		std::vector<size_t> result;
		result.reserve(_containing_query_result_moving_avg);

		float x = point.x;
		float y = point.y;
		size_t currentId = _rootNode;

		while(true)
		{
			const Node & current = _stored_nodes[currentId];
			for(size_t idx = current.start; idx < current.start + current.len; idx++)
			{
				if(_stored_boxes[idx].bb.Contains(point.x, point.y))
					result.push_back(_stored_boxes[idx].objectId);
			}

			if(current.LeafNode())
				break;

			if(x <= current.xmid)
			{
				currentId = y <= current.ymid ? current.topLeft() : current.bottomLeft();
			} else
			{
				currentId = y <= current.ymid ? current.topRight() : current.bottomRight();
			}
		};

		_containing_query_result_moving_avg = _query_avg_weight * _containing_query_result_moving_avg + (1 - _query_avg_weight) * result.size();

		return result;
	}

	std::vector<size_t> QuadTree::GetIntersectingBBIds(BoundingBox box)
	{
		std::vector<size_t> result;
		result.reserve(_intersecting_query_result_moving_avg);

		_query_queue.push(_rootNode);

		while(_query_queue.size() > 0)
		{
			Node & current = _stored_nodes[_query_queue.front()];
			_query_queue.pop();

			auto upto = current.start + current.len;
			for(size_t idx = current.start; idx < upto; idx++)
			{
				if(_stored_boxes[idx].bb.Overlaps(box))
					result.push_back(_stored_boxes[idx].objectId);
			}

			if(current.LeafNode())
			{
				continue;
			}

			if(box.xmin <= current.xmid && box.ymin <= current.ymid)
			{
				_query_queue.push(current.topLeft());
			}

			if(box.xmax >= current.xmid && box.ymin <= current.ymid)
			{
				_query_queue.push(current.topRight());
			}

			if(box.xmin <= current.xmid && box.ymax >= current.ymid)
			{
				_query_queue.push(current.bottomLeft());
			}

			if(box.xmax >= current.xmid && box.ymax >= current.ymid)
			{
				_query_queue.push(current.bottomRight());
			}
		}

		_intersecting_query_result_moving_avg = _query_avg_weight * _intersecting_query_result_moving_avg + (1 - _query_avg_weight) * result.size();

		return result;
	}

	size_t QuadTree::SmallestContainingNode(BoundingBox & box)
	{
		size_t currentId = _rootNode;

		while (true) {
			Node & current = _stored_nodes[currentId];

			if(current.LeafNode())
				break;

			if(box.xmax <= current.xmid && box.ymax <= current.ymid)
			{
				currentId = current.topLeft();
				continue;
			}

			if(box.xmin >= current.xmid && box.ymax <= current.ymid)
			{
				currentId = current.topRight();
				continue;
			}

			if(box.xmax <= current.xmid && box.ymin >= current.ymid)
			{
				currentId = current.bottomLeft();
				continue;
			}

			if(box.xmin >= current.xmid && box.ymin >= current.ymid)
			{
				currentId = current.bottomRight();
				continue;
			}

			break;
		}

		return currentId;
	}

	void QuadTree::MakeDepthWalk(std::vector<size_t>& depth_walk, size_t parent_pos, std::set<size_t>& visited) {
		if (visited.count(parent_pos)) return;
		depth_walk.push_back(parent_pos);
		visited.insert(parent_pos);
		auto & node = _stored_nodes[parent_pos];
		if (node.LeafNode() || node.firstChild ==(size_t)-1 ) return;
		MakeDepthWalk(depth_walk, node.firstChild, visited);
		MakeDepthWalk(depth_walk, node.firstChild + 1, visited);
		MakeDepthWalk(depth_walk, node.firstChild + 2, visited);
		MakeDepthWalk(depth_walk, node.firstChild + 3, visited);
	}

	void QuadTree::UpdateTree(std::vector<Box>& add_boxes, std::vector<size_t>& del_boxes)
	{
		std::map<size_t, std::vector<Box>> additions;
		std::map<size_t, std::set<size_t>> removals;

		for(auto & box : add_boxes)
		{
			size_t currentId = SmallestContainingNode(box.bb);

			if(additions.find(currentId) == additions.end())
			{
				additions.insert({ currentId, std::vector<Box>() });
			}

			additions[currentId].push_back(box);
		}

		for(auto boxId : del_boxes)
		{
			size_t idFound = 0;
			bool nodeFound = false;
			for(size_t nodeId = 0; nodeId < _stored_nodes.size(); nodeId++)
			{
				Node & node = _stored_nodes[nodeId];
				for(size_t idx = node.start; idx < node.start + node.len; idx++)
				{
					if(_stored_boxes[idx].objectId == boxId)
					{
						nodeFound = true;
						idFound = nodeId;
						break;
					}
				}

				if(nodeFound) {
					break;
				}
			}

			if(!nodeFound)
				continue;

			if(removals.find(idFound) == removals.end())
			{
				removals.insert({ idFound, std::set<size_t>() });
			}

			removals[idFound].insert(boxId);
		}

		std::vector<size_t> depth_walk;
		std::set<size_t> visited;
		MakeDepthWalk(depth_walk, 0, visited);

		// For each node we will remove boxes that need to be removed and extend vector with boxes to be added
		int runningMove = 0;
		//for(size_t nodeId = 0; nodeId < _stored_nodes.size(); nodeId++)
		for (size_t nodeId : depth_walk)
		{
			auto & node = _stored_nodes[nodeId];
			node.start += runningMove;

			size_t freedSpace = 0;
			int runningDelta = 0;
			if(removals.find(nodeId) != removals.end())
			{
				auto & rems = removals[nodeId];

				size_t pos = node.start;
				while (pos < node.start + node.len - freedSpace)
				{
					size_t id = _stored_boxes[pos].objectId;

					if(rems.find(id) != rems.end())
					{
						// Just copy probably valid box from the end and overwriting the one we are deleting
						// We will check this new box on the next step
						_stored_boxes[pos] = _stored_boxes[node.start + node.len - freedSpace - 1];
						_stored_boxes[node.start + node.len - freedSpace - 1] = {{0,0,0,0}, (size_t)-1 };
						freedSpace++;
					} else
					{
						// Proceed if we are keeping this box
						pos++;
					}
				}

				runningDelta -= rems.size();
			}

			if(additions.find(nodeId) != additions.end())
			{
				auto & adds = additions[nodeId];

				// if there are slots left after previous step, fill them all
				if(freedSpace > 0)
				{
					auto upTo = (freedSpace > adds.size())? adds.end() : adds.begin() + freedSpace;
					std::copy(adds.begin(), upTo, _stored_boxes.begin() + node.start + node.len - freedSpace);
				}

				// check if need to insert remaining
				if(freedSpace < adds.size())
				{
					auto insertAt = _stored_boxes.begin() + node.start + node.len;

					// First #freedSpace elements are already copied
					_stored_boxes.insert(insertAt, adds.begin() + freedSpace, adds.end());
				}

				runningDelta += adds.size();
			}

			// if we freed more space than filled in this node, let's cut out part of the vector
			if(runningDelta < 0)
			{
				auto deleteStart = _stored_boxes.begin() + node.start + node.len + runningDelta;

				_stored_boxes.erase(deleteStart, deleteStart + abs(runningDelta));
			}

			node.len += runningDelta;
			runningMove += runningDelta;
		}
	}
}
