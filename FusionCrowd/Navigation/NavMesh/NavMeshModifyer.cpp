#include "NavMeshModifyer.h"
#include "NavMeshLocalizer.h"
#include <algorithm>

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	NavMeshModifyer::NavMeshModifyer(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer) : _localizer(localizer), _navmesh(navmesh)
	{
	}

	NavMeshModifyer::~NavMeshModifyer()
	{
	}

	/*
	TODO:
	1. vertices not on navmesh +
	2. edges crossing whole node
	3. nodes with double modification
	*/
	float NavMeshModifyer::SplitPolyByNodes(FCArray<NavMeshVetrex> & polygon) {
		float res = 0;
		_global_polygon = std::vector<Vector2>(polygon.size());
		_modifications = std::vector<NodeModificator*>();
		std::vector<unsigned int> nodes_ids = std::vector<unsigned int>(polygon.size());
		if (!IsClockwise(polygon)) {
			for (int i = 0; i < polygon.size(); i++) {
				Vector2 v = Vector2(polygon[i].X, polygon[i].Y);
				_global_polygon[i] = v;
			}
		}
		else {
			for (int i = 0; i < polygon.size(); i++) {
				Vector2 v = Vector2(polygon[i].X, polygon[i].Y);
				_global_polygon[_global_polygon.size() - i - 1] = v;
			}
		}

		for (int i = 0; i < polygon.size(); i++) {
			nodes_ids[i] = _localizer->getNodeId(_global_polygon[i]);
		}

		int start_pos = polygon.size() - 1;
		bool one_node = true;
		if (nodes_ids[0] == nodes_ids[start_pos]) {
			while (nodes_ids[0] == nodes_ids[start_pos - 1]) {
				if (nodes_ids[start_pos] == -1) one_node = false;
				start_pos--;
				if (start_pos == 0) {
					break;
				}
			}
		}
		else {
			start_pos = 0;
			one_node = false;
		}
		if (start_pos != 0) one_node = false;
		if (one_node) {
			NodeModificator* modificator = new NodeModificator();
			modificator->modification_type = CUT_POLY;
			modificator->node = GetNodeById(nodes_ids[0]);
			modificator->polygon_to_cut = _global_polygon;
			modificator->polygon_vertex_ids = std::vector<unsigned int>(_global_polygon.size());
			for (int i = 0; i < _global_polygon.size(); i++) {
				modificator->polygon_vertex_ids[i] = i + _navmesh.vCount;
			}
			_modifications.push_back(modificator);
			return 111111;
		}
		//if poly not on one node
		int i = start_pos;
		do {
			unsigned int prev_point_id = (i - 1 + _global_polygon.size()) % _global_polygon.size();
			Vector2  prev_point = _global_polygon[prev_point_id];
			Vector2  cur_point = _global_polygon[i];
			float minx = std::min(prev_point.x, cur_point.x);
			float maxx = std::max(prev_point.x, cur_point.x);
			float miny = std::min(prev_point.y, cur_point.y);
			float maxy = std::max(prev_point.y, cur_point.y);
			//check for line i-1 - i crossing whole nodes
			auto crossing_nodes_ids = _localizer->findNodesCrossingBB(BoundingBox(minx, miny, maxx, maxy));
			for (int i = 0; i < _navmesh.nCount; i++) {
				if (std::find(crossing_nodes_ids.begin(),
					crossing_nodes_ids.end(),
					_navmesh.nodes[i]._id) != crossing_nodes_ids.end()) {
					auto cross_points = FindPolyAndSegmentCrosspoints(prev_point, cur_point, &_navmesh.nodes[i]._poly);
					if (cross_points.size() == 2) {
						//create split modificator
						NodeModificator* modificator = new NodeModificator();
						modificator->modification_type = SPLIT;
						modificator->node = &_navmesh.nodes[i];
						modificator->polygon_to_cut = std::vector<Vector2>();
						modificator->polygon_to_cut.push_back(cross_points[0]);
						modificator->polygon_to_cut.push_back(cross_points[1]);
						modificator->polygon_vertex_ids = std::vector<unsigned int>();
						modificator->polygon_vertex_ids.push_back(AddVertex(cross_points[0]));
						modificator->polygon_vertex_ids.push_back(AddVertex(cross_points[1]));
						_modifications.push_back(modificator);
					}
				}
			}
			//skip vertexes not on navmesh
			if (nodes_ids[i] == -1) {
					i = (i + 1) % _global_polygon.size();
					continue;
			}
			//add curve modificator
			NodeModificator* modificator = new NodeModificator();
			modificator->modification_type = CUT_CURVE;
			modificator->node = GetNodeById(nodes_ids[i]);
			modificator->polygon_to_cut = std::vector<Vector2>();
			modificator->polygon_vertex_ids = std::vector<unsigned int>();
			Vector2 prev_cross_point = FindPolyAndSegmentCrosspoints(
				_global_polygon[prev_point_id],
				_global_polygon[i],
				&GetNodeById(nodes_ids[i])->_poly)[0];
			modificator->polygon_vertex_ids.push_back(AddVertex(prev_cross_point));
			modificator->polygon_to_cut.push_back(prev_cross_point);
			int previ = i;
			do {
				previ = i;
				modificator->polygon_vertex_ids.push_back(_navmesh.vCount + i);
				modificator->polygon_to_cut.push_back(_global_polygon[i]);
				i = (i + 1) % _global_polygon.size();
			} while (nodes_ids[i] == nodes_ids[previ]);

			Vector2 post_cross_point = FindPolyAndSegmentCrosspoints(
				_global_polygon[previ],
				_global_polygon[i],
				&GetNodeById(nodes_ids[previ])->_poly)[0];
			modificator->polygon_vertex_ids.push_back(AddVertex(post_cross_point));
			modificator->polygon_to_cut.push_back(post_cross_point);
			_modifications.push_back(modificator);
		} while (i != start_pos);
		return 	res;
	}

	NavMeshNode* NavMeshModifyer::GetNodeById(unsigned int id) {
		for (int i = 0; i < _navmesh.nCount; i++) {
			if (_navmesh.nodes[i]._id == id) return &_navmesh.nodes[i];
		}
		return &_navmesh.nodes[0];
	}

	float NavMeshModifyer::CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
		float res = -8888;
		_addedvertices = std::vector<Vector2>();
		_nodes_ids_to_delete = std::vector<unsigned int>();
		_addednodes = std::vector<NavMeshNode*>();
		SplitPolyByNodes(polygon);
		for (auto mod : _modifications) {
			Initialize(mod);
			if (mod->modification_type == CUT_CURVE) {
				FillAddedVertices();
				CutCurveFromCurrentNode();
			}
			if (mod->modification_type == CUT_POLY) {
				FillAddedVertices();
				CutPolyFromCurrentNode();
			}
			if (mod->modification_type == SPLIT) {
				res = mod->polygon_to_cut[0].x;
				SplitNode();
			}
			_nodes_ids_to_delete.push_back(mod->node->getID());
		}
		Finalize();
		return res;
	}

	int NavMeshModifyer::SplitNode() {
		auto v0 = _local_polygon[0];
		auto v1 = _local_polygon[1];
		unsigned int v0id = _local_polygon_vertex_ids[0];
		unsigned int v1id = _local_polygon_vertex_ids[1];

		NavMeshNode* node0 = new NavMeshNode(), *node1 = new NavMeshNode();
		int n0size = 2, n1size = 2;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			if (IsPointUnderLine(v0, v1, vertex))
				n0size++;
			else
				n1size++;
		}
		node0->setID(_navmesh.nCount + _addednodes.size());
		node1->setID(_navmesh.nCount + _addednodes.size() + 1);
		node0->_poly.vertCount = n0size;
		node0->_poly.vertIDs = new unsigned int[n0size];
		node1->_poly.vertCount = n1size;
		node1->_poly.vertIDs = new unsigned int[n1size];

		int added0 = 0, added1 = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			if (IsPointUnderLine(v0, v1, vertex)) {
				node0->_poly.vertIDs[added0] = _current_node_poly->vertIDs[i];
				added0++;
			}
			else {
				node1->_poly.vertIDs[added1] = _current_node_poly->vertIDs[i];
				added1++;
			}
		}
		bool addingque = IsSegmentsIntersects(
			_navmesh.vertices[node0->_poly.vertIDs[0]],v0,
			_navmesh.vertices[node0->_poly.vertIDs[1]],v1);
		if (addingque) {
			node1->_poly.vertIDs[added1] = v0id;
			node1->_poly.vertIDs[added1 + 1] = v1id;
			node0->_poly.vertIDs[added0] = v0id;
			node0->_poly.vertIDs[added0 + 1] = v1id;
		} else {
			node1->_poly.vertIDs[added1] = v1id;
			node1->_poly.vertIDs[added1 + 1] = v0id;
			node0->_poly.vertIDs[added0] = v1id;
			node0->_poly.vertIDs[added0 + 1] = v0id;
		}
		_addednodes.push_back(node0);
		_addednodes.push_back(node1);
		return 0;
	}

	/*Adds all created nodes and vertexes*/
	int NavMeshModifyer::Finalize() {
		//add created vertices
		Vector2* updvertices = new Vector2[_navmesh.vCount + _global_polygon.size() + _addedvertices.size()];
		for (int i = 0; i < _navmesh.vCount; i++) {
			updvertices[i] = _navmesh.vertices[i];
		}
		delete[] _navmesh.vertices;
		for (int i = 0; i < _global_polygon.size(); i++) {
			updvertices[_navmesh.vCount + i] = _global_polygon[i];
		}
		for (int i = 0; i < _addedvertices.size(); i++) {
			updvertices[_navmesh.vCount + _global_polygon.size() + i] = _addedvertices[i];
		}
		_navmesh.vCount = _navmesh.vCount + _global_polygon.size() + _addedvertices.size();
		_navmesh.vertices = updvertices;

		for (auto n : _addednodes) {
			n->setVertices(_navmesh.vertices);
		}

		//add nodes
		NavMeshNode* tmpNodes = new NavMeshNode[_navmesh.nCount + _addednodes.size() - _nodes_ids_to_delete.size()];
		int deleted = 0;
		for (size_t i = 0; i < _navmesh.nCount; i++)
		{
			if (std::find(
				_nodes_ids_to_delete.begin(),
				_nodes_ids_to_delete.end(),
				_navmesh.nodes[i]._id) == _nodes_ids_to_delete.end()) {
				tmpNodes[i - deleted] = _navmesh.nodes[i];
			}
			else {
				deleted++;
			}
		}
		delete[] _navmesh.nodes;
		for (int i = 0; i < _addednodes.size(); i++) {
			tmpNodes[_navmesh.nCount + i - _nodes_ids_to_delete.size()] = *_addednodes[i];
		}
		_navmesh.nCount += _addednodes.size() - _nodes_ids_to_delete.size();
		_navmesh.nodes = tmpNodes;
		return 0;
	}

	/*Creates arrays*/
	int NavMeshModifyer::Initialize(NodeModificator * modificator) {
		_current_node_poly = &modificator->node->_poly;
		_local_polygon = modificator->polygon_to_cut;
		_local_polygon_vertex_ids = modificator->polygon_vertex_ids;
		return 0;
	}

	/*Cut whole poligon from onde node*/
	int NavMeshModifyer::CutPolyFromCurrentNode() {
		for (int j = 0; j < _local_polygon.size(); j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(_navmesh.nCount + _addednodes.size());
			int vert_count = 0;
			Vector2 j0vert = _local_polygon[j];
			Vector2 j1vert = _local_polygon[(j + 1) % _local_polygon.size()];
			Vector2 j2vert = _local_polygon[(j + 2) % _local_polygon.size()];
			bool node_side0 = !IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			vert_count += 4;
			updnode->_poly.vertCount = vert_count;
			updnode->_poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					updnode->_poly.vertIDs[addedids] = _current_node_poly->vertIDs[i];
					addedids++;
				}
				if (_current_node_poly->vertIDs[i] == crosspoints_prev_vertex_ids[j]) {
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[j];//j0j1 crosspoint
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 1) % _local_polygon.size()]; //polygon node j1
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 2) % _local_polygon.size()]; //polygon node j2
					addedids++;
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[(j + 1) % crosspoints_ids.size()]; //j1j2 crosspoint
					addedids++;
				}
			}
			if (addedids != vert_count) return 88;
			//TODO: obstacles and edges
			_addednodes.push_back(updnode);
		}
		return 0;
	}

	int NavMeshModifyer::CutCurveFromCurrentNode() {
		for (int j = 0; j < _local_polygon.size() -2; j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(_navmesh.nCount + _addednodes.size());
			int vert_count = 0;
			Vector2 j0vert = _local_polygon[j];
			Vector2 j1vert = _local_polygon[(j + 1) % _local_polygon.size()];
			Vector2 j2vert = _local_polygon[(j + 2) % _local_polygon.size()];
			bool node_side0 = !IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			vert_count += 4;
			updnode->_poly.vertCount = vert_count;
			updnode->_poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;

			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
				bool vertex_added = false;
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					updnode->_poly.vertIDs[addedids] = _current_node_poly->vertIDs[i];
					addedids++;
					vertex_added = true;
				}
				if (_current_node_poly->vertIDs[i] == crosspoints_prev_vertex_ids[j]) {
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[j];//j0j1 crosspoint
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 1) % _local_polygon_vertex_ids.size()]; //polygon node j1
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 2) % _local_polygon_vertex_ids.size()]; //polygon node j2
					addedids++;
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[(j+1)%crosspoints_ids.size()]; //j1j2 crosspoint
					addedids++;
				}
			}

			//TODO: obstacles and edges
			_addednodes.push_back(updnode);
		}

		//add node splited by v0 v1 line

		NavMeshNode* updnode = new NavMeshNode();
		updnode->setID(_navmesh.nCount + _local_polygon.size() - 2);
		int vert_count = 0;
		Vector2 j0vert = _local_polygon[0];
		Vector2 j1vert = _local_polygon[1];
		Vector2 j2vert = _local_polygon[2];
		bool node_side0 = IsPointUnderLine(j0vert, j1vert, j2vert);
		//for correct vertex adding
		std::vector<Vector2> tmpvertex = std::vector<Vector2>();
		int post_index = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				vert_count++;
				tmpvertex.push_back(vertex);
			}
			if (_current_node_poly->vertIDs[i] == crosspoints_prev_vertex_ids[_local_polygon.size() - 1]) {
				post_index = vert_count;
			}
		}
		vert_count += 3;
		updnode->_poly.vertCount = vert_count;
		updnode->_poly.vertIDs = new unsigned int[vert_count];

		Vector2 prev = tmpvertex[(post_index - 1 + tmpvertex.size()) % tmpvertex.size()];
		Vector2 post = tmpvertex[post_index % tmpvertex.size()];
		bool que = !IsSegmentsIntersects(prev, _local_polygon[1], _local_polygon[0], post);

		int addedids = 0;
		int res =post.x==6 && post.y == 1?7 :8;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			bool added = false;
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) ) {
				updnode->_poly.vertIDs[addedids] = _current_node_poly->vertIDs[i];
				added = true;
				addedids++;
			}
			if (_current_node_poly->vertIDs[i] == crosspoints_prev_vertex_ids[_local_polygon.size() - 1]) {

				//float k = -1;
				//if (j0vert.x != j1vert.x) k = (j0vert.y - j1vert.y) / (j0vert.x - j1vert.x);
				//bool que = !(IsTriangleClockwise(vertex, _local_polygon[1], _local_polygon[0]) != added);
				//if (k < 0) que = !que;
				if (!added) que = !que;
				if (que) {
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[0]; //j0j1 crosspoint
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[1]; //polygon node j1
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[0]; //polygon node j0
					addedids++;
				}
				else {
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[0]; //polygon node j0
					addedids++;
					updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[1]; //polygon node j1
					addedids++;
					updnode->_poly.vertIDs[addedids] = crosspoints_ids[0]; //j0j1 crosspoint
					addedids++;

				}
			}
		}
		_addednodes.push_back(updnode);

		return res;
	}

	/*Returns cross point with current_poly in dirrection v0->v1*/
	Vector2 NavMeshModifyer::FindVortexCrossPoint(Vector2 v0, Vector2 v1, int& out_prev_cross_id) {
		//calculate line coef
		float k0 = 0.0;
		float c0 = 0.0;
		bool vertical = false;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k0 = (v0.y - v1.y) / (v0.x - v1.x);
			c0 = v0.y - k0 * v0.x;
		}
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			Vector2 pnode0 = Vector2();
			Vector2 pnode1 = Vector2();
			if (i < _current_node_poly->vertCount - 1) {
				pnode0.x = _current_node_poly->vertices[_current_node_poly->vertIDs[i]].x;
				pnode0.y = _current_node_poly->vertices[_current_node_poly->vertIDs[i]].y;
				pnode1.x = _current_node_poly->vertices[_current_node_poly->vertIDs[i + 1]].x;
				pnode1.y = _current_node_poly->vertices[_current_node_poly->vertIDs[i + 1]].y;
			}
			else {
				pnode0.x = _current_node_poly->vertices[_current_node_poly->vertIDs[i]].x;
				pnode0.y = _current_node_poly->vertices[_current_node_poly->vertIDs[i]].y;
				pnode1.x = _current_node_poly->vertices[_current_node_poly->vertIDs[0]].x;
				pnode1.y = _current_node_poly->vertices[_current_node_poly->vertIDs[0]].y;
			}

			//line cross point claculation
			float xcross = 0.0;
			float ycross = 0.0;
			if (vertical) {
				if (pnode0.x == pnode1.x) continue;
				float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
				float c1 = pnode0.y - k1 * pnode0.x;
				xcross = v0.x;
				ycross = k1 * xcross + c1;
			} else {
				if (pnode0.x == pnode1.x) {
					xcross = pnode0.x;
					ycross = k0 * xcross + c0;
				}
				else {
					float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
					if (k1 == k0) continue;
					float c1 = pnode0.y - k1 * pnode0.x;

					xcross = (c1 - c0) / (k0 - k1);
					ycross = k1 * xcross + c1;
				}
			}

			//is cross point on segment?
			if (
				((xcross>=pnode0.x && xcross<=pnode1.x) ||
				(xcross<=pnode0.x && xcross>=pnode1.x)) &&
				((ycross >= pnode0.y && ycross <= pnode1.y) ||
				(ycross <= pnode0.y && ycross >= pnode1.y))
				) {
				//is cross point in v0->v1 direction?
				if (
					(v1.x > v0.x && xcross >= v1.x) ||
					(v1.x < v0.x && xcross <= v1.x) ||
					(v1.x == v0.x && v1.y> v0.y && ycross >= v1.y) ||
					(v1.x == v0.x && v1.y < v0.y && ycross <= v1.y)
					){
					out_prev_cross_id = _current_node_poly->vertIDs[i];
					return Vector2(xcross, ycross);
				}
				else continue;
			}
		}
		return Vector2(0, 0);
	}

	/*Adds crosspoints for polygon cut*/
	void NavMeshModifyer::FillAddedVertices() {
		crosspoints_prev_vertex_ids = std::vector<unsigned int>(_local_polygon.size());
		crosspoints_ids = std::vector<unsigned int>(_local_polygon.size());
		int prev_cross_id = 0;
		for (int i = 0; i < _local_polygon.size(); i++) {
			Vector2 crosspoint = FindVortexCrossPoint(_local_polygon[i], _local_polygon[(i + 1) % _local_polygon.size()], prev_cross_id);
			crosspoints_ids[i] = AddVertex(crosspoint);
			crosspoints_prev_vertex_ids[i] = prev_cross_id;
		}
	}

	/*Returns is point under line v0->v1 (or point.x< line.x if line is vertical)*/
	bool NavMeshModifyer::IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse, bool strict) {
		bool vertical = false;
		float k = 0.0;
		float c = 0.0;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k = (v0.y - v1.y) / (v0.x - v1.x);
			c = v0.y - k * v0.x;
		}
		if (vertical) {
			if (strict)
				return reverse ? point.x > v0.x : point.x < v0.x;
			else
				return reverse ? point.x >= v0.x : point.x <= v0.x;
		}
		if (strict) {
			if (reverse)
				return point.y > k*point.x + c;
			else
				return point.y < k*point.x + c;
		} else
			if (reverse)
				return point.y >= k*point.x + c;
			else
				return point.y <= k*point.x + c;
	}

	bool NavMeshModifyer::IsClockwise(FCArray<NavMeshVetrex> & polygon) {
		float sum = 0;
		for (int i = 0; i < polygon.size(); i++) {
			NavMeshVetrex v0 = polygon[i];
			NavMeshVetrex v1 = polygon[(i+1) % polygon.size()];
			sum += (v1.X-v0.X)*(v0.Y + v1.Y);
		}

		return sum > 0;
	}

	bool NavMeshModifyer::IsTriangleClockwise(Vector2 v0, Vector2 v1, Vector2 v2){
		float sum = 0;
		sum += (v1.x - v0.x)*(v0.y + v1.y);
		sum += (v2.x - v1.x)*(v1.y + v2.y);
		sum += (v0.x - v2.x)*(v2.y + v0.y);
		return sum > 0;

	}

	//adds vertex and returns its ID
	unsigned int NavMeshModifyer::AddVertex(Vector2 v) {
		_addedvertices.push_back(v);
		return _navmesh.vCount + _global_polygon.size() + _addedvertices.size() - 1;
	}

	std::vector<Vector2> NavMeshModifyer::FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly) {
		std::vector<Vector2> res = std::vector<Vector2>();
		//calculate line coef
		float k0 = 0.0;
		float c0 = 0.0;
		bool vertical = false;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k0 = (v0.y - v1.y) / (v0.x - v1.x);
			c0 = v0.y - k0 * v0.x;
		}
		for (int i = 0; i < poly->vertCount; i++) {
			Vector2 pnode0 = Vector2();
			Vector2 pnode1 = Vector2();
			if (i < poly->vertCount - 1) {
				pnode0.x = poly->vertices[poly->vertIDs[i]].x;
				pnode0.y = poly->vertices[poly->vertIDs[i]].y;
				pnode1.x = poly->vertices[poly->vertIDs[i + 1]].x;
				pnode1.y = poly->vertices[poly->vertIDs[i + 1]].y;
			}
			else {
				pnode0.x = poly->vertices[poly->vertIDs[i]].x;
				pnode0.y = poly->vertices[poly->vertIDs[i]].y;
				pnode1.x = poly->vertices[poly->vertIDs[0]].x;
				pnode1.y = poly->vertices[poly->vertIDs[0]].y;
			}

			//line cross point claculation
			float xcross = 0.0;
			float ycross = 0.0;
			if (vertical) {
				if (pnode0.x == pnode1.x) continue;
				float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
				float c1 = pnode0.y - k1 * pnode0.x;
				xcross = v0.x;
				ycross = k1 * xcross + c1;
			}
			else {
				if (pnode0.x == pnode1.x) {
					xcross = pnode0.x;
					ycross = k0 * xcross + c0;
				}
				else {
					float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
					if (k1 == k0) continue;
					float c1 = pnode0.y - k1 * pnode0.x;

					xcross = (c1 - c0) / (k0 - k1);
					ycross = k1 * xcross + c1;
				}
			}

			//is cross point on poly segment?
			if (
				((xcross >= pnode0.x && xcross <= pnode1.x) ||
				(xcross <= pnode0.x && xcross >= pnode1.x)) &&
					((ycross >= pnode0.y && ycross <= pnode1.y) ||
				(ycross <= pnode0.y && ycross >= pnode1.y))
				) {
				//is cross point on v0v1 segment?
				if (
					((xcross >= v0.x && xcross <= v1.x) ||
					(xcross <= v0.x && xcross >= v1.x)) &&
						((ycross >= v0.y && ycross <= v1.y) ||
					(ycross <= v0.y && ycross >= v1.y))
					) {
					res.push_back(Vector2(xcross, ycross));
				}
			}
		}
		return res;
	}

	bool NavMeshModifyer::IsSegmentsIntersects(Vector2 v00, Vector2 v01, Vector2 v10, Vector2 v11) {
		//calculate line coef
		float k0 = 0.0;
		float c0 = 0.0;
		bool vertical = false;
		if (v00.x == v01.x) {
			vertical = true;
		}
		else {
			k0 = (v00.y - v01.y) / (v00.x - v01.x);
			c0 = v00.y - k0 * v00.x;
		}
		Vector2 pnode0 = v10;
		Vector2 pnode1 = v11;
		//line cross point claculation
		float xcross = 0.0;
		float ycross = 0.0;
		if (vertical) {
			if (pnode0.x == pnode1.x) return false;
			float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
			float c1 = pnode0.y - k1 * pnode0.x;
			xcross = v00.x;
			ycross = k1 * xcross + c1;
		}
		else {
			if (pnode0.x == pnode1.x) {
				xcross = pnode0.x;
				ycross = k0 * xcross + c0;
			}
			else {
				float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
				if (k1 == k0) return false;
				float c1 = pnode0.y - k1 * pnode0.x;

				xcross = (c1 - c0) / (k0 - k1);
				ycross = k1 * xcross + c1;
			}
		}

		//is cross point on poly segment?
		if (
			((xcross >= pnode0.x && xcross <= pnode1.x) ||
			(xcross <= pnode0.x && xcross >= pnode1.x)) &&
				((ycross >= pnode0.y && ycross <= pnode1.y) ||
			(ycross <= pnode0.y && ycross >= pnode1.y))
			) {
			//is cross point on v0v1 segment?
			if (
				((xcross >= v00.x && xcross <= v01.x) ||
				(xcross <= v00.x && xcross >= v01.x)) &&
					((ycross >= v00.y && ycross <= v01.y) ||
				(ycross <= v00.y && ycross >= v01.y))
				) {
				return true;
			}
			else return false;
		}
	}
}
