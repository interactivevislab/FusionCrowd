#include "NavMeshModifyer.h"
#include "NavMeshLocalizer.h"
#include <algorithm>

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	NavMeshModifyer::NavMeshModifyer(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query)
		: _localizer(localizer), _navmesh(navmesh), _spatial_query(spatial_query)
	{
	}

	NavMeshModifyer::~NavMeshModifyer()
	{
	}

	float NavMeshModifyer::SplitPolyByNodes(FCArray<NavMeshVetrex> & polygon) {
		float res = 0;
		_global_polygon = std::vector<Vector2>(polygon.size());
		_modifications = std::vector<NodeModificator*>();
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
		FixConcavePoly();
		std::vector<unsigned int> nodes_ids = std::vector<unsigned int>(_global_polygon.size());
		if (_global_polygon.size() < 3) return 0;
		for (int i = 0; i < _global_polygon.size(); i++) {
			NavMeshNode* node = FindNodeByPoint(_global_polygon[i]);
			if (node != nullptr) {
				nodes_ids[i] = node->_id;
			}
			else {
				nodes_ids[i] = NavMeshLocation::NO_NODE;
			}
		}

#pragma region start_pos_proccess
		int start_pos = 0;
		bool one_node = true;
		for (auto id : nodes_ids) {
			if (id != nodes_ids[0]) {
				one_node = false;
				break;
			}
		}
		if (!one_node) {
			while (nodes_ids[start_pos] == nodes_ids[(start_pos - 1 + _global_polygon.size()) % _global_polygon.size()]) {
				start_pos = (start_pos - 1 + _global_polygon.size()) % _global_polygon.size();
			}
		}
#pragma endregion

		if (one_node) {
			if (nodes_ids[0] != NavMeshLocation::NO_NODE) {
				NodeModificator* modificator = new NodeModificator();
				modificator->modification_type = CUT_POLY;
				modificator->node = GetNodeById(nodes_ids[0]);
				modificator->polygon_to_cut = _global_polygon;
				modificator->polygon_vertex_ids = std::vector<unsigned int>(_global_polygon.size());
				for (int i = 0; i < _global_polygon.size(); i++) {
					modificator->polygon_vertex_ids[i] = i + _navmesh.vCount;
				}
				_modifications.push_back(modificator);
				return 1;
			}
		}
		//if poly not on one node
		int i = start_pos;
		do {
			unsigned int prev_point_id = (i - 1 + _global_polygon.size()) % _global_polygon.size();
			Vector2 prev_point = _global_polygon[prev_point_id];
			Vector2 cur_point = _global_polygon[i];
			Vector2 next_point = _global_polygon[(i + 1) % _global_polygon.size()];
			float minx = std::min(prev_point.x, cur_point.x);
			float maxx = std::max(prev_point.x, cur_point.x);
			float miny = std::min(prev_point.y, cur_point.y);
			float maxy = std::max(prev_point.y, cur_point.y);

#pragma region node_split_modificators
			//check for line i-1 - i crossing whole nodes
			auto crossing_nodes_ids = _localizer->findNodesCrossingBB(BoundingBox(minx, miny, maxx, maxy));
			for (int j = 0; j < _navmesh.nCount; j++) {
				if (_navmesh.nodes[j].deleted) continue;
				if (std::find(crossing_nodes_ids.begin(),
					crossing_nodes_ids.end(),
					_navmesh.nodes[j]._id) != crossing_nodes_ids.end()) {
					auto cross_points = FindPolyAndSegmentCrosspoints(prev_point, cur_point, &_navmesh.nodes[j]._poly);
					if (cross_points.size() == 2) {
						//create split modificator
						NodeModificator* modificator = new NodeModificator();
						modificator->modification_type = SPLIT;
						modificator->node = &_navmesh.nodes[j];
						modificator->polygon_to_cut = std::vector<Vector2>();
						modificator->polygon_to_cut.push_back(cross_points[0]);
						modificator->polygon_to_cut.push_back(cross_points[1]);
						modificator->polygon_vertex_ids = std::vector<unsigned int>();
						modificator->polygon_vertex_ids.push_back(AddVertex(cross_points[0]));
						modificator->polygon_vertex_ids.push_back(AddVertex(cross_points[1]));
						modificator->side = !IsPointUnderLine(cross_points[0], cross_points[1], next_point);
						_modifications.push_back(modificator);
					}
				}
			}
#pragma endregion

			//skip vertexes not on navmesh
			if (nodes_ids[i] == NavMeshLocation::NO_NODE) {
				i = (i + 1) % _global_polygon.size();
				continue;
			}

#pragma region curve_modificator
			//add curve modificator
			NodeModificator* modificator = new NodeModificator();
			modificator->modification_type = CUT_CURVE;
			modificator->node = GetNodeById(nodes_ids[i]);
			modificator->polygon_to_cut = std::vector<Vector2>();
			modificator->polygon_vertex_ids = std::vector<unsigned int>();
			auto nodeptr = &modificator->node->_poly;

			auto tmp = FindPolyAndSegmentCrosspoints(
				_global_polygon[prev_point_id],
				_global_polygon[i],
				nodeptr);
			//TODO fix
			if (tmp.size() == 0) {
				delete modificator;
				tres = -1;
				int previ = i;
				do {
					previ = i;
					i = (i + 1) % _global_polygon.size();
				} while (nodes_ids[i] == nodes_ids[previ]);
				continue;
			}
			Vector2 prev_cross_point = tmp[0];
			modificator->polygon_vertex_ids.push_back(AddVertex(prev_cross_point));
			modificator->polygon_to_cut.push_back(prev_cross_point);
			int previ = i;
			do {
				previ = i;
				modificator->polygon_vertex_ids.push_back(_navmesh.vCount + i);
				modificator->polygon_to_cut.push_back(_global_polygon[i]);
				i = (i + 1) % _global_polygon.size();
			} while (nodes_ids[i] == nodes_ids[previ]);

			auto cross_points = FindPolyAndSegmentCrosspoints(
				_global_polygon[previ],
				_global_polygon[i],
				nodeptr);
			//TODO fix
			if (cross_points.size() == 0) {
				delete modificator;
				tres = -2;
				continue;
			}
			Vector2 post_cross_point = cross_points[0];

			modificator->polygon_vertex_ids.push_back(AddVertex(post_cross_point));
			modificator->polygon_to_cut.push_back(post_cross_point);
			_modifications.push_back(modificator);
#pragma endregion

		} while (i != start_pos);
		return 	res;
	}

	NavMeshNode* NavMeshModifyer::GetNodeById(size_t id) {
		for (int i = 0; i < _navmesh.nCount; i++) {
			if (_navmesh.nodes[i]._id == id) return &_navmesh.nodes[i];
		}
		for (auto n : _addednodes) {
			if (n->_id == id) return n;
		}
		return nullptr;
	}

	float NavMeshModifyer::CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
		_addedvertices = std::vector<Vector2>();
		_nodes_ids_to_delete = std::vector<size_t>();
		_addednodes = std::vector<NavMeshNode*>();
		_addededges = std::vector<NavMeshEdge*>();
		_addedobstacles = std::vector<NavMeshObstacle*>();

		SplitPolyByNodes(polygon);
		if (_global_polygon.size() < 3) return -1;
		for (auto mod : _modifications) {
			Initialize(mod);
			if (mod->modification_type == CUT_CURVE) {
				FillAddedVertices(true);
				CutCurveFromCurrentNode();
			}
			if (mod->modification_type == CUT_POLY) {
				FillAddedVertices(false);
				CutPolyFromCurrentNode();
			}
			if (mod->modification_type == SPLIT) {
				SplitNode();
			}
			if (std::find(_nodes_ids_to_delete.begin(), _nodes_ids_to_delete.end(), mod->node->getID())
				== _nodes_ids_to_delete.end()) {
				_nodes_ids_to_delete.push_back(mod->node->getID());
			}
		}

		if (_modifications.size() == 0) return tres != 0 ? tres : -2;
		//TODO remove
		if (tres < 0) return tres;
		tres = _modifications.size();
		Finalize();
		return tres;
	}

	int NavMeshModifyer::SplitNode() {
		auto v0 = _local_polygon[0];
		auto v1 = _local_polygon[1];
		unsigned int v0id = _local_polygon_vertex_ids[0];
		unsigned int v1id = _local_polygon_vertex_ids[1];

#pragma region node_creation
		NavMeshNode* updnode = new NavMeshNode();
		int vert_count = 2;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			auto next_vertex = _navmesh.vertices[_current_node_poly->vertIDs[(i + 1) % _current_node_poly->vertCount]];
			bool vert_added = false;
			if (IsPointUnderLine(v0, v1, vertex) == _side) {
				vert_count++;
				vert_added = true;
			}
		}
		updnode->setID(GetNextNodeID());
		updnode->_poly.vertCount = vert_count;
		updnode->_poly.vertIDs = new unsigned int[vert_count];

#pragma endregion

		int added0 = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			bool addingque = false;
			if (IsPointUnderLine(v0, v1, vertex) == _side) {
				updnode->_poly.vertIDs[added0] = _current_node_poly->vertIDs[i];
				addingque = true;
				added0++;
			}
		}
		updnode->_poly.vertIDs[added0] = v1id;
		added0++;
		updnode->_poly.vertIDs[added0] = v0id;
		added0++;

		//copy obstacles
#pragma region copy_obstacles
		std::vector<NavMeshObstacle*> node0_obst = std::vector<NavMeshObstacle*>(),
			node1_obst = std::vector<NavMeshObstacle*>();
		for (int i = 0; i < _current_node->_obstCount; i++) {
			auto obst = _current_node->_obstacles[i];
			Vector2 ov0 = obst->_point;
			Vector2 ov1 = ov0 + (obst->_length*(obst->_unitDir));
			bool ov0side = IsPointUnderLine(v0, v1, ov0) == _side;
			bool ov1side = IsPointUnderLine(v0, v1, ov1) == _side;
			if (ov0side && ov1side) {
				NavMeshObstacle* nobst = new NavMeshObstacle();
				nobst->_point = obst->_point;
				nobst->_unitDir = obst->_unitDir;
				nobst->_length = obst->_length;
				if (ov0side) {
					nobst->setNode(updnode);
					node0_obst.push_back(nobst);
				}
			}
			else if (ov0side || ov1side) {
				Vector2 divpoint;
				if (ov0.x == ov1.x) {
					if (v0.x == ov0.x) divpoint = v0;
					else divpoint = v1;
				}
				else {
					float k = (ov0.y - ov1.y) / (ov0.x - ov1.x);
					float c = ov0.y - k * ov0.x;
					if (v0.y == k * v0.x + c) divpoint = v0;
					else divpoint = v1;
				}
				NavMeshObstacle* nobst = new NavMeshObstacle();
				if (ov0side) {
					nobst->_point = ov0;
					nobst->_unitDir = obst->_unitDir;
					nobst->_length = (divpoint - ov0).Length();
				}
				else {
					nobst->_point = divpoint;
					nobst->_unitDir = obst->_unitDir;
					nobst->_length = (ov1 - divpoint).Length();
				}
				nobst->setNode(updnode);
				node0_obst.push_back(nobst);
			}
		}
		NavMeshObstacle* nobst0 = new NavMeshObstacle();
		nobst0->_point = v0;
		nobst0->_unitDir = (v1 - v0) / (v1 - v0).Length();
		nobst0->_length = (v1 - v0).Length();
		nobst0->setNode(updnode);
		node0_obst.push_back(nobst0);

		updnode->_obstacles = new NavMeshObstacle*[node0_obst.size()];
		updnode->_obstCount = node0_obst.size();
		for (int i = 0; i < node0_obst.size(); i++) {
			updnode->_obstacles[i] = node0_obst[i];
			_addedobstacles.push_back(node0_obst[i]);
		}
#pragma endregion
		//copy edges
#pragma region copy_edges
		for (int i = 0; i < _current_node->_edgeCount; i++) {
			auto edge = _current_node->_edges[i];
			Vector2 ev0 = edge->getP0();
			Vector2 ev1 = edge->getP1();
			bool ev0side = IsPointUnderLine(v0, v1, ev0) == _side;
			bool ev1side = IsPointUnderLine(v0, v1, ev1) == _side;
			if (ev0side && ev1side) {
				NavMeshEdge* nedge = new NavMeshEdge();
				nedge->setPoint(edge->getP0());
				nedge->setDirection(edge->getDirection());
				nedge->setWidth(edge->getWidth());
				_addededges.push_back(nedge);
				if (ev0side) {
					nedge->setNodes(updnode, nullptr);
				}
			}
			else if (ev0side || ev1side) {
				Vector2 divpoint;
				if (ev0.x == ev1.x) {
					if (v0.x == ev0.x) divpoint = v0;
					else divpoint = v1;
				}
				else {
					float k = (ev0.y - ev1.y) / (ev0.x - ev1.x);
					float c = ev0.y - k * ev0.x;
					if (v0.y == k * v0.x + c) divpoint = v0;
					else divpoint = v1;
				}
				NavMeshEdge* nedge = new NavMeshEdge();
				if (ev0side) {
					nedge->setPoint(ev0);
					nedge->setDirection(edge->getDirection());
					nedge->setWidth((ev0 - divpoint).Length());
				}
				else {
					nedge->setPoint(divpoint);
					nedge->setDirection(edge->getDirection());
					nedge->setWidth((ev1 - divpoint).Length());
				}
				nedge->setNodes(updnode, nullptr);
				_addededges.push_back(nedge);
			}
		}
#pragma endregion

		_addednodes.push_back(updnode);
		return 0;
	}

	/*Adds all created nodes and vertexes*/
	int NavMeshModifyer::Finalize() {


#pragma region vertices
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

		for (int i = 0; i < _navmesh.nCount; i++) {
			_navmesh.nodes[i].setVertices(_navmesh.vertices);
		}
#pragma endregion

#pragma region copy_old_edges

		std::vector<NavMeshEdge> vtmp_edges = std::vector<NavMeshEdge>();
		for (int i = 0; i < _navmesh.eCount; i++) {
			NavMeshEdge& edge = _navmesh.edges[i];
			unsigned int first_node_id = edge.getFirstNode()->_id;
			unsigned int second_node_id = edge.getSecondNode()->_id;
			NavMeshNode* first_node = nullptr;
			NavMeshNode* second_node = nullptr;
			if (std::find(
				_nodes_ids_to_delete.begin(),
				_nodes_ids_to_delete.end(),
				first_node_id)
				== _nodes_ids_to_delete.end()) {
				first_node = edge.getFirstNode();
			}
			if (std::find(
				_nodes_ids_to_delete.begin(),
				_nodes_ids_to_delete.end(),
				second_node_id)
				== _nodes_ids_to_delete.end()) {
				second_node = edge.getSecondNode();
			}
			if (first_node != nullptr && second_node != nullptr) {
				vtmp_edges.push_back(edge);
			}
			else {
				if (first_node != nullptr || second_node != nullptr) {
					NavMeshEdge* nedge = new NavMeshEdge();
					nedge->setNodes(first_node, second_node);
					nedge->setPoint(edge.getP0());
					nedge->setDirection(edge.getDirection());
					nedge->setWidth(edge.getWidth());
					_addededges.push_back(nedge);
				}
			}
		}

#pragma endregion

		std::vector<unsigned int> first_nodes_ids = std::vector<unsigned int>(vtmp_edges.size());
		std::vector<unsigned int> second_nodes_ids = std::vector<unsigned int>(vtmp_edges.size());
		for (int i = 0; i < vtmp_edges.size(); i++) {
			first_nodes_ids[i] = vtmp_edges[i].getFirstNode()->_id;
			second_nodes_ids[i] = vtmp_edges[i].getSecondNode()->_id;
		}


#pragma region obstacles_copy
		//add and delete obstacles
		std::vector<NavMeshObstacle> tmp_obstacles = std::vector<NavMeshObstacle>();
		size_t oid = 0;
		for (auto o : _navmesh.obstacles) {
			if (std::find(
				_nodes_ids_to_delete.begin(),
				_nodes_ids_to_delete.end(),
				o.getNode()->_id) == _nodes_ids_to_delete.end()) {
				o._id = oid;
				oid++;
				tmp_obstacles.push_back(o);
			}

		}

#pragma endregion

		std::vector<unsigned int> obtacle_nodes_ids = std::vector<unsigned int>(_navmesh.obstacles.size());
		for (int i = 0; i < _navmesh.obstacles.size(); i++) {
			obtacle_nodes_ids[i] = _navmesh.obstacles[i].getNode()->_id;
		}

		FinalizeNodes();
		_localizer->Update(_addednodes, _nodes_ids_to_delete);

		for (int i = 0; i < vtmp_edges.size(); i++) {
			NavMeshNode* n0 = _navmesh.GetNodeByID(first_nodes_ids[i]);
			NavMeshNode* n1 = _navmesh.GetNodeByID(second_nodes_ids[i]);
			vtmp_edges[i].setNodes(n0, n1);
		}


		for (int i = 0; i < _navmesh.obstacles.size(); i++) {
			NavMeshNode* n = _navmesh.GetNodeByID(obtacle_nodes_ids[i]);
			_navmesh.obstacles[i].setNode(n);
		}

#pragma region edge_obstacles_process

		//fill not seted nodes pointers or replace with obstacles
		for (int i = _addededges.size() - 1; i >= 0; i--) {
			if (!ProcessEdge(_addededges[i])) {
				delete _addededges[i];
				_addededges[i] = nullptr;
			}
		}
		_addededges.erase(std::remove(_addededges.begin(), _addededges.end(), nullptr), _addededges.end());


		//delete duplicated edges
		for (int i = _addededges.size() - 1; i >= 0; i--) {
			auto it = _addededges[i];
			for (auto edge : _addededges) {
				if (edge != it &&
					((edge->getFirstNode()->_id == it->getFirstNode()->_id && edge->getSecondNode()->_id == it->getSecondNode()->_id) ||
					(edge->getFirstNode()->_id == it->getSecondNode()->_id && edge->getSecondNode()->_id == it->getFirstNode()->_id))) {
					if (it->getWidth() >= edge->getWidth()) {
						delete it;
						_addededges.erase(_addededges.begin() + i);
						break;
					}
				}
			}
		}

#pragma endregion


#pragma region add_new_obsts

		for (int i = 0; i < _addedobstacles.size(); i++) {
			_addedobstacles[i]->_id = oid;
			oid++;
			tmp_obstacles.push_back(*_addedobstacles[i]);
		}

		for (int i = 0; i < _addedobstacles.size(); i++) {
			delete _addedobstacles[i];
		}
		_navmesh.obstCount = tmp_obstacles.size();
		_navmesh.obstacles = tmp_obstacles;

#pragma endregion

#pragma region add_new_edges
		//add and delete edges
		for (int i = 0; i < _addededges.size(); i++) {
			vtmp_edges.push_back(*_addededges[i]);
		}

		delete[] _navmesh.edges;
		_navmesh.edges = new NavMeshEdge[vtmp_edges.size()];
		for (int i = 0; i < vtmp_edges.size(); i++) {
			_navmesh.edges[i] = vtmp_edges[i];
		}
		_navmesh.eCount = vtmp_edges.size();
#pragma endregion

#pragma region fill_edges_obst_arrays
		//fill obst and edges nodes arrays
		for (int i = 0; i < _navmesh.nCount; i++) {
			NavMeshNode& node = _navmesh.nodes[i];
			if (node.deleted) continue;
			size_t node_id = node._id;
			delete[] node._edges;
			delete[] node._obstacles;
			std::vector<NavMeshEdge*> edges = std::vector<NavMeshEdge*>();
			std::vector<NavMeshObstacle*> obstacles = std::vector<NavMeshObstacle*>();
			for (int ei = 0; ei < _navmesh.eCount; ei++) {
				if (_navmesh.edges[ei].getFirstNode()->_id == node_id ||
					_navmesh.edges[ei].getSecondNode()->_id == node_id) {
					edges.push_back(&_navmesh.edges[ei]);
					if (_navmesh.edges[ei].getFirstNode()->_id == node_id) {
						_navmesh.edges[ei].setNodes(&node, _navmesh.edges[ei].getSecondNode());
					}
					else {
						_navmesh.edges[ei].setNodes(_navmesh.edges[ei].getFirstNode(), &node);
					}
				}
			}
			for (int oi = 0; oi < _navmesh.obstCount; oi++) {
				if (_navmesh.obstacles[oi].getNode()->_id == node_id) {
					obstacles.push_back(&_navmesh.obstacles[oi]);
					_navmesh.obstacles[oi].setNode(&node);
				}
			}
			node._obstCount = obstacles.size();
			node._obstacles = new NavMeshObstacle*[node._obstCount];
			for (int i = 0; i < node._obstCount; i++) {
				node._obstacles[i] = obstacles[i];
			}
			node._edgeCount = edges.size();
			node._edges = new NavMeshEdge*[node._edgeCount];
			for (int i = 0; i < node._edgeCount; i++) {
				node._edges[i] = edges[i];
			}
		}

#pragma endregion


		for (auto m : _modifications) {
			delete m;
		}

		for (int i = 0; i < _addednodes.size(); i++) {
			delete _addednodes[i];
		}

		_spatial_query->Update();

		return 0;
	}

	/*Creates arrays*/
	int NavMeshModifyer::Initialize(NodeModificator * modificator) {
		_current_node_poly = &modificator->node->_poly;
		_current_node = modificator->node;
		_local_polygon = modificator->polygon_to_cut;
		_local_polygon_vertex_ids = modificator->polygon_vertex_ids;
		_side = modificator->side;
		return 0;
	}

	/*Cut whole poligon from onde node*/
	int NavMeshModifyer::CutPolyFromCurrentNode() {
		NavMeshNode* prev_node = nullptr;
		NavMeshEdge* first_edge = nullptr;
		for (int j = 0; j < _local_polygon.size(); j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(GetNextNodeID());
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
			}
			updnode->_poly.vertIDs[addedids] = crosspoints_ids[j];//j0j1 crosspoint
			addedids++;
			updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 1) % _local_polygon.size()]; //polygon node j1
			addedids++;
			updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 2) % _local_polygon.size()]; //polygon node j2
			addedids++;
			updnode->_poly.vertIDs[addedids] = crosspoints_ids[(j + 1) % crosspoints_ids.size()]; //j1j2 crosspoint
			addedids++;

#pragma region add_edges
			NavMeshEdge* edge = new NavMeshEdge();
			Vector2 crosspoint = crosspoints[j];
			edge->setPoint(j1vert);
			edge->setWidth((crosspoint - j1vert).Length());
			edge->setDirection((crosspoint - j1vert) / (crosspoint - j1vert).Length());
			edge->setNodes(prev_node, updnode);
			_addededges.push_back(edge);
			if (prev_node == nullptr) first_edge = edge;
			prev_node = updnode;
			CopyVortexObstacles(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);
			CopyVortexEdges(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);

			_addednodes.push_back(updnode);
#pragma endregion

		}
		first_edge->setNodes(prev_node, first_edge->getSecondNode());
		return _addedobstacles.size();
	}

	int NavMeshModifyer::CutCurveFromCurrentNode() {
		NavMeshNode* prev_node = nullptr;
		NavMeshEdge* first_edge = nullptr;
		for (int j = 0; j < _local_polygon.size() - 2; j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(GetNextNodeID());
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
			if (j < _local_polygon.size() - 3) { vert_count++; }
			vert_count += 3;
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
			}
			updnode->_poly.vertIDs[addedids] = crosspoints_ids[j];//j0j1 crosspoint
			addedids++;
			updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 1) % _local_polygon_vertex_ids.size()]; //polygon node j1
			addedids++;
			updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[(j + 2) % _local_polygon_vertex_ids.size()]; //polygon node j2
			addedids++;
			if (j < _local_polygon.size() - 3) {
				updnode->_poly.vertIDs[addedids] = crosspoints_ids[(j + 1) % crosspoints_ids.size()]; //j1j2 crosspoint
				addedids++;
			}

#pragma region create_edge
			NavMeshEdge* edge = new NavMeshEdge();
			Vector2 crosspoint = crosspoints[j];
			edge->setPoint(j1vert);
			edge->setWidth((crosspoint - j1vert).Length());
			edge->setDirection((crosspoint - j1vert) / (crosspoint - j1vert).Length());
			edge->setNodes(prev_node, updnode);

			_addededges.push_back(edge);
			if (prev_node == nullptr) first_edge = edge;
			prev_node = updnode;
#pragma endregion
			CopyVortexObstacles(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);
			CopyVortexEdges(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);
			_addednodes.push_back(updnode);
		}

		//add node splited by v0 v1 line
#pragma region add_first_node
		NavMeshNode* updnode = new NavMeshNode();
		updnode->setID(GetNextNodeID());
		int vert_count = 0;
		Vector2 j0vert = _local_polygon[0];
		Vector2 j1vert = _local_polygon[1];
		Vector2 j2vert = _local_polygon[2];
		bool node_side0 = IsPointUnderLine(j0vert, j1vert, j2vert);
		//for correct vertex adding
		int post_index = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				vert_count++;
			}
		}
		vert_count += 3;
		updnode->_poly.vertCount = vert_count;
		updnode->_poly.vertIDs = new unsigned int[vert_count];


		int addedids = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_node_poly->vertIDs[i]];
			bool added = false;
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				updnode->_poly.vertIDs[addedids] = _current_node_poly->vertIDs[i];
				added = true;
				addedids++;
			}
		}
		updnode->_poly.vertIDs[addedids] = crosspoints_ids[0]; //j0j1 crosspoint
		addedids++;
		updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[1]; //polygon node j1
		addedids++;
		updnode->_poly.vertIDs[addedids] = _local_polygon_vertex_ids[0]; //polygon node j0
		addedids++;

		CopyVortexObstacles(updnode, -1, j0vert, j1vert, j2vert, node_side0, false, true);
		CopyVortexEdges(updnode, -1, j0vert, j1vert, j2vert, node_side0, false, true);
		first_edge->setNodes(updnode, first_edge->getSecondNode());

		_addednodes.push_back(updnode);
#pragma endregion

		return 0;
	}

	/*Returns cross point with current_poly in dirrection v0->v1*/
	Vector2 NavMeshModifyer::FindVortexCrossPoint(Vector2 v0, Vector2 v1, bool& success) {
		success = true;
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
			Vector2 pnode0 = _current_node_poly->vertices[_current_node_poly->vertIDs[i]];
			Vector2 pnode1 = _current_node_poly->vertices[_current_node_poly->vertIDs[(i + 1) % _current_node_poly->vertCount]];

#pragma region crosspoint_calculation
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
#pragma endregion

			//is cross point on segment?
			if (
				((xcross >= pnode0.x && xcross <= pnode1.x) ||
				(xcross <= pnode0.x && xcross >= pnode1.x)) &&
					((ycross >= pnode0.y && ycross <= pnode1.y) ||
				(ycross <= pnode0.y && ycross >= pnode1.y))
				) {
				//is cross point in v0->v1 direction?
				if (
					(v1.x > v0.x && xcross >= v1.x) ||
					(v1.x < v0.x && xcross <= v1.x) ||
					(v1.x == v0.x && v1.y > v0.y && ycross >= v1.y) ||
					(v1.x == v0.x && v1.y < v0.y && ycross <= v1.y)
					) {
					return Vector2(xcross, ycross);
				}
				else continue;
			}
		}
		//TODO remove
		success = false;
		tres = -999;
		return Vector2(999999, 999999);
	}

	/*Adds crosspoints for polygon cut*/
	void NavMeshModifyer::FillAddedVertices(bool isCurve) {
		int max = isCurve ? _local_polygon.size() - 2 : _local_polygon.size();
		crosspoints_ids = std::vector<unsigned int>(max);
		crosspoints = std::vector<Vector2>(max);
		for (int i = 0; i < max; i++) {
			bool f;
			Vector2 crosspoint = FindVortexCrossPoint(_local_polygon[i], _local_polygon[(i + 1) % _local_polygon.size()], f);
			crosspoints_ids[i] = AddVertex(crosspoint);
			crosspoints[i] = crosspoint;
		}
	}

	/*Returns is point under line v0->v1 (or point.x< line.x if line is vertical)*/
	bool NavMeshModifyer::IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse, bool strict) {
		bool vertical = false;
		bool res;
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
			if (strict) {
				res = point.x < v0.x;
			}
			else {
				res = point.x <= v0.x;
			}
		}
		else {
			if (strict) {
				res = point.y < k*point.x + c;
			}
			else {
				res = point.y <= k * point.x + c;
			}
		}

		return reverse ? !res : res;
	}

	bool NavMeshModifyer::IsClockwise(FCArray<NavMeshVetrex> & polygon) {
		float sum = 0;
		for (int i = 0; i < polygon.size(); i++) {
			NavMeshVetrex v0 = polygon[i];
			NavMeshVetrex v1 = polygon[(i + 1) % polygon.size()];
			sum += (v1.X - v0.X)*(v0.Y + v1.Y);
		}

		return sum > 0;
	}

	bool NavMeshModifyer::IsTriangleClockwise(Vector2 v0, Vector2 v1, Vector2 v2) {
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

#pragma region crosspoint_calculation
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

#pragma endregion

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

	void NavMeshModifyer::CopyVortexObstacles(NavMeshNode* updnode, int j,
		Vector2 j0vert, Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict) {

		std::vector<NavMeshObstacle*> tmp_obst = std::vector<NavMeshObstacle*>();
		NavMeshObstacle *obst = new NavMeshObstacle();
		obst->setNode(updnode);
		Vector2 p0 = _local_polygon[(j + 1) % _local_polygon.size()];
		Vector2 p1 = _local_polygon[(j + 2) % _local_polygon.size()];
		obst->_point = p0;
		obst->_unitDir = (p1 - p0) / (p1 - p0).Length();
		obst->_length = (p1 - p0).Length();
		tmp_obst.push_back(obst);

		for (int i = 0; i < _current_node->_obstCount; i++) {
			NavMeshObstacle *obst = _current_node->_obstacles[i];
			Vector2 p0 = obst->getP0();
			Vector2 p1 = obst->getP1();
			bool p0side = IsPointUnderLine(j0vert, j1vert, p0, node_side0, true) &&
				IsPointUnderLine(j1vert, j2vert, p0, node_side1);
			bool p1side = IsPointUnderLine(j0vert, j1vert, p1, node_side0, true) &&
				IsPointUnderLine(j1vert, j2vert, p1, node_side1);
			if (onestrict) {
				p0side = IsPointUnderLine(j0vert, j1vert, p0, node_side0, true);
				p1side = IsPointUnderLine(j0vert, j1vert, p1, node_side0, true);
			}
			if (p0side && p1side) {
				obst->setNode(updnode);
				NavMeshObstacle *nobst = new NavMeshObstacle();
				nobst->setNode(updnode);
				nobst->_point = p0;
				nobst->_unitDir = (p1 - p0) / (p1 - p0).Length();
				nobst->_length = (p1 - p0).Length();
				tmp_obst.push_back(nobst);
			}
			else {
				Vector2 v0 = crosspoints[j];
				Vector2 v1 = crosspoints[(j + 1) % crosspoints.size()];
				if (onestrict) {
					v0 = j0vert;
				}
				if (p0side || p1side) {
#pragma region one_x_on_obst
					Vector2 divpoint;
					if (p0.x == p1.x) {
						if (v1.x == p0.x) divpoint = v1;
						else if (v0.x == p0.x) {
							divpoint = v0;
						}
						else divpoint = _local_polygon[_local_polygon.size() - 1];
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v0.y == k * v0.x + c) divpoint = v0;
						else if (v1.y == k * v1.x + c) {
							divpoint = v1;
						}
						else divpoint = _local_polygon[_local_polygon.size() - 1];
					}
					NavMeshObstacle *nobst = new NavMeshObstacle();
					nobst->setNode(updnode);
					nobst->_point = divpoint;
					tmp_obst.push_back(nobst);
					if (p0side) {
						nobst->_unitDir = (p0 - divpoint) / (p0 - divpoint).Length();
						nobst->_length = (p0 - divpoint).Length();
					}
					else {
						nobst->_unitDir = (p1 - divpoint) / (p1 - divpoint).Length();
						nobst->_length = (p1 - divpoint).Length();
					}
#pragma endregion
				}
				else {
#pragma region both_x_on_obst
					//is both crosspoints on edge?
					bool create_obst = true;
					if (p0.x == p1.x) {
						if (v0.x != p0.x) create_obst = false;
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v0.y != k * v0.x + c) create_obst = false;
					}
					if (p0.x == p1.x) {
						if (v1.x != p0.x) create_obst = false;
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v1.y != k * v1.x + c) create_obst = false;
					}
					if (create_obst) {
						NavMeshObstacle *nobst = new NavMeshObstacle();
						nobst->setNode(updnode);
						nobst->_point = v0;
						nobst->_unitDir = (v1 - v0) / (v1 - v0).Length();
						nobst->_length = (v1 - v0).Length();
						tmp_obst.push_back(nobst);
					}
#pragma endregion

				}
			}
		}

		updnode->_obstacles = new NavMeshObstacle*[tmp_obst.size()];
		updnode->_obstCount = tmp_obst.size();
		for (int i = 0; i < tmp_obst.size(); i++) {
			updnode->_obstacles[i] = tmp_obst[i];
			_addedobstacles.push_back(tmp_obst[i]);
		}
	}


	void NavMeshModifyer::CopyVortexEdges(NavMeshNode* updnode, int j,
		Vector2 j0vert, Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict) {

		std::vector<NavMeshEdge*> tmp_edges = std::vector<NavMeshEdge*>();

		for (int i = 0; i < _current_node->_edgeCount; i++) {
			NavMeshEdge *edge = _current_node->_edges[i];
			Vector2 p0 = edge->getP0();
			Vector2 p1 = edge->getP1();
			bool p0side = IsPointUnderLine(j0vert, j1vert, p0, node_side0, true) &&
				IsPointUnderLine(j1vert, j2vert, p0, node_side1);
			bool p1side = IsPointUnderLine(j0vert, j1vert, p1, node_side0, true) &&
				IsPointUnderLine(j1vert, j2vert, p1, node_side1);
			if (onestrict) {
				p0side = IsPointUnderLine(j0vert, j1vert, p0, node_side0, true);
				p1side = IsPointUnderLine(j0vert, j1vert, p1, node_side0, true);
			}
			if (p0side && p1side) {
				NavMeshEdge* nedge = new NavMeshEdge();
				nedge->setNodes(updnode, nullptr);
				nedge->setPoint(edge->getP0());
				nedge->setDirection(edge->getDirection());
				nedge->setWidth(edge->getWidth());
				tmp_edges.push_back(nedge);
			}
			else {
				Vector2 v0 = crosspoints[j];
				Vector2 v1 = crosspoints[(j + 1) % crosspoints.size()];
				if (onestrict) {
					v0 = j0vert;
				}
				if (p0side || p1side) {
					Vector2 divpoint;
					if (p0.x == p1.x) {
						if (v0.x == p0.x) divpoint = v0;
						else divpoint = v1;
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v0.y == k * v0.x + c) divpoint = v0;
						else divpoint = v1;
					}
					NavMeshEdge *nedge = new NavMeshEdge();
					nedge->setNodes(updnode, nullptr);
					nedge->setPoint(divpoint);
					tmp_edges.push_back(nedge);
					if (p0side) {
						nedge->setDirection((p0 - divpoint) / (p0 - divpoint).Length());
						nedge->setWidth((p0 - divpoint).Length());
					}
					else {
						nedge->setDirection((p1 - divpoint) / (p1 - divpoint).Length());
						nedge->setWidth((p1 - divpoint).Length());
					}
				}
				else {
					//is both crosspoints on edge?
					bool create_edge = true;
					if (p0.x == p1.x) {
						if (v0.x != p0.x) create_edge = false;
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v0.y != k * v0.x + c) create_edge = false;
					}
					if (p0.x == p1.x) {
						if (v1.x != p0.x) create_edge = false;
					}
					else {
						float k = (p0.y - p1.y) / (p0.x - p1.x);
						float c = p0.y - k * p0.x;
						if (v1.y != k * v1.x + c) create_edge = false;
					}
					if (create_edge) {
						NavMeshEdge *nedge = new NavMeshEdge();
						nedge->setNodes(updnode, nullptr);
						nedge->setPoint(v0);
						nedge->setDirection((v1 - v0) / (v1 - v0).Length());
						nedge->setWidth((v1 - v0).Length());
						tmp_edges.push_back(nedge);
					}
				}
			}
		}

		updnode->_edges = new NavMeshEdge*[tmp_edges.size()];
		updnode->_edgeCount = tmp_edges.size();
		for (int i = 0; i < tmp_edges.size(); i++) {
			updnode->_edges[i] = tmp_edges[i];
			_addededges.push_back(tmp_edges[i]);
		}
	}

	/*Find second/first node if it == nullptr, adds obstacle if node can't be finded*/
	bool NavMeshModifyer::ProcessEdge(NavMeshEdge* edge) {
		if (edge->getFirstNode() != nullptr && edge->getSecondNode() != nullptr) {
			return true;
		}

		float d = 0.25;
		Vector2 mid_edge = edge->getP0(edge->getWidth() / 2.0);
		Vector2 check_point;
		size_t exist_id = 0;
		NavMeshNode* node;
		if (edge->getFirstNode() == nullptr) {
			node = edge->getSecondNode();
			check_point = mid_edge - edge->getSecondNode()->getCenter();
			exist_id = edge->getSecondNode()->_id;
		}
		else {
			node = edge->getFirstNode();
			check_point = mid_edge - edge->getFirstNode()->getCenter();
			exist_id = edge->getFirstNode()->_id;
		}
		check_point.Normalize();
		check_point *= d;
		check_point += mid_edge;
		size_t node_id = _localizer->getNodeId(check_point);
		if (node_id == exist_id) return false; //TODO it shouldn't happend
		if (node_id == NavMeshLocation::NO_NODE) {

			NavMeshObstacle* obst = new NavMeshObstacle();
			obst->setNode(edge->getFirstNode() == nullptr ? edge->getSecondNode() : edge->getFirstNode());
			obst->_point = edge->getP0();
			obst->_unitDir = edge->getDirection();
			obst->_length = edge->getWidth();
			_addedobstacles.push_back(obst);
			return false;
		}
		if (edge->getFirstNode() == nullptr) {
			edge->setNodes(GetNodeById(node_id), edge->getSecondNode());
		}
		else {
			edge->setNodes(edge->getFirstNode(), GetNodeById(node_id));
		}
		return true;
	}

	unsigned int NavMeshModifyer::GetNextNodeID() {
		return _navmesh.nCount + _addednodes.size();
	}

	void NavMeshModifyer::FixPoly(NavMeshNode& node) {
		Vector2 last_added;
		std::vector<size_t> ids_left = std::vector<size_t>(node._poly.vertCount);
		std::vector<size_t> res = std::vector<size_t>(node._poly.vertCount);

		Vector2 mean = Vector2(0, 0);
		for (int i = 0; i < node._poly.vertCount; i++) {
			ids_left[i] = node._poly.vertIDs[i];
			mean += node._poly.vertices[node._poly.vertIDs[i]];
		}
		mean /= node._poly.vertCount;

		Vector2 prev_dir = node._poly.vertices[ids_left[0]] - mean;
		prev_dir.Normalize();
		size_t added = 0;
		while (ids_left.size() > 0) {
			float max_product = -FLT_MAX;
			int candidateidpos = 0;
			for (int i = 0; i < ids_left.size(); i++) {
				Vector2 mean_v_vec = node._poly.vertices[ids_left[i]] - mean;
				mean_v_vec.Normalize();
				float dotpr = prev_dir.Dot(mean_v_vec);
				float dir = mean_v_vec.x * prev_dir.y - mean_v_vec.y * prev_dir.x;
				if (dotpr > max_product && dir > 0) {
					max_product = dotpr;
					candidateidpos = i;
				}
			}
			res[added] = ids_left[candidateidpos];
			prev_dir = node._poly.vertices[ids_left[candidateidpos]] - mean;
			prev_dir.Normalize();
			added++;
			ids_left.erase(ids_left.begin() + candidateidpos);
		}

		//fill res
		for (int i = 0; i < res.size(); i++) {
			node._poly.vertIDs[i] = res[i];
		}
	}

	void NavMeshModifyer::FixConcavePoly() {
		//remove close points
		std::vector<float> distances = std::vector<float>(_global_polygon.size());
		for (int i = 0; i < _global_polygon.size(); i++) {
			distances[i] = Vector2::Distance(_global_polygon[i], _global_polygon[(i + 1) % _global_polygon.size()]);
		}
		for (int i = _global_polygon.size() - 1; i >= 0; i--) {
			if (distances[i] < 0.5f) {
				_global_polygon.erase(_global_polygon.begin() + i);
			}
		}
		int pos = 0, neg = 0;
		do {
			do {
				std::vector<float> results = std::vector<float>(_global_polygon.size());
				pos = 0; neg = 0;
				for (int i = 0; i < _global_polygon.size(); i++) {
					Vector2 v0 = _global_polygon[(i + _global_polygon.size() - 1) % _global_polygon.size()];
					Vector2 v1 = _global_polygon[i];
					Vector2 v2 = _global_polygon[(i + 1) % _global_polygon.size()];
					float res = (v1.x - v0.x)*(v2.y - v1.y) - (v1.y - v0.y)*(v2.x - v1.x);
					results[i] = res;
					if (res > 0) pos++;
					else neg++;
				}
				bool cut_neg = neg < pos;
				for (int i = results.size() - 1; i >= 0; i--) {
					if ((cut_neg && results[i] < 0) || (!cut_neg && results[i] > 0)) {
						_global_polygon.erase(_global_polygon.begin() + i);
					}
					else {
						if (results[i] == 0) {
							_global_polygon.erase(_global_polygon.begin() + i);
						}
					}
				}
			} while (pos > 0 && neg > 0);
			FixGlobalPoly();
		} while (pos > 0 && neg > 0);

		//remove smooth lines
		std::vector<bool> delete_mark = std::vector<bool>(_global_polygon.size());
		for (int i = 0; i < _global_polygon.size(); i++) {
			Vector2 v0 = _global_polygon[(i + 1) % _global_polygon.size()] - _global_polygon[i];
			Vector2 v1 = _global_polygon[(i + 2) % _global_polygon.size()] - _global_polygon[(i + 1) % _global_polygon.size()];
			v0.Normalize();
			v1.Normalize();
			if (v0.Dot(v1) > 0.996f) {
				delete_mark[(i + 1) % _global_polygon.size()] = true;
			}
			else {
				delete_mark[(i + 1) % _global_polygon.size()] = false;

			}

		}
		for (int i = _global_polygon.size() - 1; i >= 0; i--) {
			if (delete_mark[i]) {
				_global_polygon.erase(_global_polygon.begin() + i);
			}
		}
	}

	void NavMeshModifyer::FinalizeNodes() {
		for (auto n : _addednodes) {
			FixPoly(*n);
			Vector2 center = Vector2(0, 0);
			for (int i = 0; i < n->_poly.vertCount; i++) {
				center += _navmesh.vertices[n->_poly.vertIDs[i]];
			}
			n->_poly.vertices = _navmesh.vertices;
			center /= (float)n->_poly.vertCount;
			n->setCenter(center);
			n->_poly.setBB();
		}
		NavMeshNode* tmpNodes = new NavMeshNode[_navmesh.nCount + _addednodes.size()];
		for (size_t i = 0; i < _navmesh.nCount; i++)
		{
			tmpNodes[i] = _navmesh.nodes[i];
			if (std::find(
				_nodes_ids_to_delete.begin(),
				_nodes_ids_to_delete.end(),
				_navmesh.nodes[i]._id) != _nodes_ids_to_delete.end()) {
				tmpNodes[i].deleted = true;
				for (int j = 0; j < _navmesh.nodes[i]._edgeCount; j++) {
					NavMeshEdge* e = _navmesh.nodes[i]._edges[j];
					if (e->getFirstNode() != nullptr
						&& e->getFirstNode()->getID() == _navmesh.nodes[i].getID()) {
						e->setNodes(nullptr, e->getSecondNode());
					}
					if (e->getSecondNode() != nullptr
						&& e->getSecondNode()->getID() == _navmesh.nodes[i].getID()) {
						e->setNodes(e->getFirstNode(), nullptr);
					}
				}
			}
		}
		delete[] _navmesh.nodes;
		for (int i = 0; i < _addednodes.size(); i++) {
			tmpNodes[_navmesh.nCount + i] = *_addednodes[i];
		}
		_navmesh.nCount += _addednodes.size();
		_navmesh.nodes = tmpNodes;
	}

	NavMeshNode* NavMeshModifyer::FindNodeByPoint(Vector2 point) {
		const float X = point.x;
		const float Y = point.y;
		for (int i = 0; i < _navmesh.nCount; i++) {
			if (_navmesh.nodes[i].deleted) continue;
			NavMeshPoly& poly = _navmesh.nodes[i]._poly;
			poly.setBB();
			bool tf = true;
			int count = 0; // number of intersections
			for (size_t e = 0; e < poly.vertCount; ++e)
			{
				const Vector2& p0 = poly.vertices[poly.vertIDs[e]];
				if (p0 != Vector2(0, 0)) tf = false;
				const size_t next = (e + 1) % poly.vertCount;
				const Vector2& p1 = poly.vertices[poly.vertIDs[next]];
				// simple cases in which there can be no intersection
				if ((p0.y > Y && p1.y >= Y) || // polysegment above line
					(p0.y < Y && p1.y <= Y) || // polysegment below line
					(p0.x > X && p1.x > X)) // polysegment to right of test line
				{
					continue;
				}

				bool f = true;
				if (p0.x != p1.x) {
					float k = (p0.y - p1.y) / (p0.x - p1.x);
					float c = p0.y - k * p0.x;
					float x_intersect = (Y - c) / k;
					if (x_intersect > X) f = false;
				}

				if (f)
				{
					++count;
				}
			}
			if ((count % 2) == 1) return &_navmesh.nodes[i];
		}
		return nullptr;
	}

	void NavMeshModifyer::FixGlobalPoly() {
		Vector2 last_added;
		std::vector<Vector2> vert_left = std::vector<Vector2>(_global_polygon.size());
		std::vector<Vector2> res = std::vector<Vector2>(_global_polygon.size());

		Vector2 mean = Vector2(0, 0);
		for (int i = 0; i < _global_polygon.size(); i++) {
			vert_left[i] = _global_polygon[i];
			mean += _global_polygon[i];
		}
		mean /= _global_polygon.size();

		Vector2 prev_dir = _global_polygon[0] - mean;
		prev_dir.Normalize();
		size_t added = 0;
		while (vert_left.size() > 0) {
			float max_product = -FLT_MAX;
			int candidateidpos = 0;
			for (int i = 0; i < vert_left.size(); i++) {
				Vector2 mean_v_vec = _global_polygon[i] - mean;
				mean_v_vec.Normalize();
				float dotpr = prev_dir.Dot(mean_v_vec);
				float dir = mean_v_vec.x * prev_dir.y - mean_v_vec.y * prev_dir.x;
				if (dotpr > max_product && dir > 0) {
					max_product = dotpr;
					candidateidpos = i;
				}
			}
			res[added] = vert_left[candidateidpos];
			prev_dir = vert_left[candidateidpos] - mean;
			prev_dir.Normalize();
			added++;
			vert_left.erase(vert_left.begin() + candidateidpos);
		}

		//fill res
		for (int i = 0; i < res.size(); i++) {
			_global_polygon[i] = res[i];
		}
	}
}
