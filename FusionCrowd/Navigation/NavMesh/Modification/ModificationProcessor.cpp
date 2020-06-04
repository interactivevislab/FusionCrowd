#include "ModificationProcessor.h"
#include "../NavMeshLocalizer.h"
#include "ModificationHelper.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd {
	ModificationProcessor::ModificationProcessor(NavMesh& navmesh,
		std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query)
		:_modification(navmesh, localizer, spatial_query)
	{
	}

	ModificationProcessor::~ModificationProcessor()
	{
		for (auto m : _node_modificators) {
			delete m;
		}
	}

	float ModificationProcessor::CutPolygonFromMesh(std::vector<Vector2>& polygon) {
		if (polygon.size() < 3) return 0;
		Clear();
		SplitPolyByNodes(polygon);

		if (_modification._global_polygon.size() < 3) return -1;
		for (int i = 0; i < _node_modificators.size(); i++) {
			auto mod = _node_modificators[i];
			if (mod->modification_type == CUT_CURVE) {
				if (!ModificationHelper::ValidateModificator(mod, _node_modificators)) continue;
				Initialize(mod);
				FillAddedVertices(true);
				CutCurveFromCurrentNode();
			}

			if (mod->modification_type == CUT_POLY) {
				Initialize(mod);
				FillAddedVertices(false);
				CutPolyFromCurrentNode();
			}

			if (mod->modification_type == SPLIT) {
				Initialize(mod);
				SplitNode();
			}

			if (std::find(_modification._nodes_ids_to_delete.begin(), _modification._nodes_ids_to_delete.end(), mod->node->getID())
				== _modification._nodes_ids_to_delete.end()) {
				_modification._nodes_ids_to_delete.push_back(mod->node->getID());
			}

		}

		int mod_num = 0;
		for (auto m : _node_modificators) {
			if (m->correct) mod_num++;
		}
		if (mod_num == 0) return tres - 10000;

		_modification.Finalize();

		return mod_num;
	}

	void ModificationProcessor::Clear() {
		for (auto m : _node_modificators) {
			delete m;
		}
		_node_modificators.clear();
		_current_node_poly = nullptr;
		_current_node = nullptr;
		_local_polygon.clear();
		_local_polygon_vertex_ids.clear();
	}

	float ModificationProcessor::SplitPolyByNodes(std::vector<Vector2>& polygon) {
		unsigned int global_poly_size = polygon.size();
		float res = 0;
		_modification._global_polygon = std::vector<Vector2>(polygon.size());
		_node_modificators = std::vector<NodeModificator*>();
		if (!ModificationHelper::IsClockwise(polygon)) {
			for (int i = 0; i < polygon.size(); i++) {
				Vector2 v = Vector2(polygon[i].x, polygon[i].y);
				_modification._global_polygon[i] = v;
			}
		}
		else {
			for (int i = 0; i < polygon.size(); i++) {
				Vector2 v = Vector2(polygon[i].x, polygon[i].y);
				_modification._global_polygon[global_poly_size - i - 1] = v;
			}
		}
		ModificationHelper::SimplifyPoly(_modification._global_polygon);
		global_poly_size = _modification._global_polygon.size();
		std::vector<unsigned int> nodes_ids = std::vector<unsigned int>(global_poly_size);
		if (global_poly_size < 3) return 0;
		for (int i = 0; i < global_poly_size; i++) {
			NavMeshNode* node = _modification.GetGlobalPolygonNodeByVertexId(i);
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
			while (nodes_ids[start_pos] == nodes_ids[(start_pos - 1 + global_poly_size) % global_poly_size]) {
				start_pos = (start_pos - 1 + global_poly_size) % global_poly_size;
			}
		}
#pragma endregion

		if (one_node) {
			if (nodes_ids[0] != NavMeshLocation::NO_NODE) {
				NodeModificator* modificator = new NodeModificator();
				modificator->modification_type = CUT_POLY;
				modificator->node =_modification._navmesh.GetNodeByID(nodes_ids[0]);
				modificator->polygon_to_cut = _modification._global_polygon;
				modificator->polygon_vertex_ids = std::vector<unsigned int>(global_poly_size);
				for (int i = 0; i < global_poly_size; i++) {
					modificator->polygon_vertex_ids[i] = i + _modification._navmesh.vCount;
				}
				_node_modificators.push_back(modificator);
				return 1;
			}
		}
		//if poly not on one node
		int i = start_pos;
		do {
			unsigned int prev_point_id = (i - 1 + global_poly_size) % global_poly_size;
			Vector2 prev_point = _modification._global_polygon[prev_point_id];
			Vector2 cur_point = _modification._global_polygon[i];
			Vector2 next_point = _modification._global_polygon[(i + 1) % global_poly_size];
			float minx = std::min(prev_point.x, cur_point.x);
			float maxx = std::max(prev_point.x, cur_point.x);
			float miny = std::min(prev_point.y, cur_point.y);
			float maxy = std::max(prev_point.y, cur_point.y);

#pragma region node_split_modificators
			//check for line i-1 - i crossing whole nodes
			auto crossing_nodes_ids = _modification._localizer->findNodesCrossingBB(BoundingBox(minx, miny, maxx, maxy));
			for (int j = 0; j < _modification._navmesh.nCount; j++) {
				if (_modification._navmesh.nodes[j].deleted) continue;
				if (std::find(crossing_nodes_ids.begin(),
					crossing_nodes_ids.end(),
					_modification._navmesh.nodes[j]._id) != crossing_nodes_ids.end()) {
					auto cross_points = ModificationHelper::FindPolyAndSegmentCrosspoints(prev_point,
						cur_point, &_modification._navmesh.nodes[j]._poly);
					if (cross_points.size() == 2) {
						//create split modificator
						NodeModificator* modificator = new NodeModificator();
						modificator->modification_type = SPLIT;
						modificator->node = &_modification._navmesh.nodes[j];
						modificator->polygon_to_cut = std::vector<Vector2>();
						modificator->polygon_to_cut.push_back(cross_points[0]);
						modificator->polygon_to_cut.push_back(cross_points[1]);
						modificator->polygon_vertex_ids = std::vector<unsigned int>();
						modificator->polygon_vertex_ids.push_back(_modification.AddVertex(cross_points[0]));
						modificator->polygon_vertex_ids.push_back(_modification.AddVertex(cross_points[1]));
						modificator->side = !ModificationHelper::IsPointUnderLine(cross_points[0], cross_points[1], next_point);
						_node_modificators.push_back(modificator);
					}
				}
			}
#pragma endregion

			//skip vertexes not on navmesh
			if (nodes_ids[i] == NavMeshLocation::NO_NODE) {
				i = (i + 1) % global_poly_size;
				continue;
			}

#pragma region curve_modificator
			//add curve modificator
			NodeModificator* modificator = new NodeModificator();
			modificator->modification_type = CUT_CURVE;
			modificator->node = _modification._navmesh.GetNodeByID(nodes_ids[i]);
			modificator->polygon_to_cut = std::vector<Vector2>();
			modificator->polygon_vertex_ids = std::vector<unsigned int>();
			auto nodeptr = &modificator->node->_poly;

			auto tmp = ModificationHelper::FindPolyAndSegmentCrosspoints(
				_modification._global_polygon[prev_point_id],
				_modification._global_polygon[i],
				nodeptr);
			if (tmp.size() > 0) {
				Vector2 prev_cross_point = tmp[0];
				modificator->polygon_vertex_ids.push_back(_modification.AddVertex(prev_cross_point));
				modificator->polygon_to_cut.push_back(prev_cross_point);
			}
			int previ = i;
			do {
				previ = i;
				modificator->polygon_vertex_ids.push_back(_modification._navmesh.vCount + i);
				modificator->polygon_to_cut.push_back(_modification._global_polygon[i]);
				i = (i + 1) % global_poly_size;
			} while (nodes_ids[i] == nodes_ids[previ]);

			auto cross_points = ModificationHelper::FindPolyAndSegmentCrosspoints(
				_modification._global_polygon[previ],
				_modification._global_polygon[i],
				nodeptr);
			if (cross_points.size() > 0) {
				Vector2 post_cross_point = cross_points[0];
				modificator->polygon_vertex_ids.push_back(_modification.AddVertex(post_cross_point));
				modificator->polygon_to_cut.push_back(post_cross_point);
			}
			if (modificator->polygon_to_cut.size() >= 2) {
				_node_modificators.push_back(modificator);
			}
			else {
				delete modificator;
			}
#pragma endregion

		} while (i != start_pos);
		return 	res;
	}

	int ModificationProcessor::Initialize(NodeModificator * modificator) {
		_current_node_poly = &modificator->node->_poly;
		_current_node = modificator->node;
		_local_polygon = modificator->polygon_to_cut;
		_local_polygon_vertex_ids = modificator->polygon_vertex_ids;
		_side = modificator->side;
		return 0;
	}

	int ModificationProcessor::SplitNode() {
		auto v0 = _local_polygon[0];
		auto v1 = _local_polygon[1];
		unsigned int v0id = _local_polygon_vertex_ids[0];
		unsigned int v1id = _local_polygon_vertex_ids[1];

#pragma region node_creation
		NavMeshNode* updnode = new NavMeshNode();
		int vert_count = 2;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
			auto next_vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[(i + 1) % _current_node_poly->vertCount]];
			bool vert_added = false;
			if (ModificationHelper::IsPointUnderLine(v0, v1, vertex) == _side) {
				vert_count++;
				vert_added = true;
			}
		}
		updnode->setID(_modification.GetNextNodeID());
		updnode->_poly.vertCount = vert_count;
		updnode->_poly.vertIDs = new unsigned int[vert_count];

#pragma endregion

		int added0 = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
			bool addingque = false;
			if (ModificationHelper::IsPointUnderLine(v0, v1, vertex) == _side) {
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
			bool ov0side = ModificationHelper::IsPointUnderLine(v0, v1, ov0) == _side;
			bool ov1side = ModificationHelper::IsPointUnderLine(v0, v1, ov1) == _side;
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
				if (nobst->_length > _modification.min_width) {
					node0_obst.push_back(nobst);
				}
				else {
					delete nobst;
				}
			}
		}
		NavMeshObstacle* nobst0 = new NavMeshObstacle();
		nobst0->_point = v0;
		nobst0->_unitDir = (v1 - v0) / (v1 - v0).Length();
		nobst0->_length = (v1 - v0).Length();
		nobst0->setNode(updnode);
		if (nobst0->_length > _modification.min_width) {
			node0_obst.push_back(nobst0);
		}
		else {
			delete nobst0;
		}

		updnode->_obstacles = new NavMeshObstacle*[node0_obst.size()];
		updnode->_obstCount = node0_obst.size();
		for (int i = 0; i < node0_obst.size(); i++) {
			updnode->_obstacles[i] = node0_obst[i];
			_modification._addedobstacles.push_back(node0_obst[i]);
		}
#pragma endregion
		//copy edges
#pragma region copy_edges
		for (int i = 0; i < _current_node->_edgeCount; i++) {
			auto edge = _current_node->_edges[i];
			Vector2 ev0 = edge->getP0();
			Vector2 ev1 = edge->getP1();
			bool ev0side = ModificationHelper::IsPointUnderLine(v0, v1, ev0) == _side;
			bool ev1side = ModificationHelper::IsPointUnderLine(v0, v1, ev1) == _side;
			if (ev0side && ev1side) {
				NavMeshEdge* nedge = new NavMeshEdge();
				nedge->setPoint(edge->getP0());
				nedge->setDirection(edge->getDirection());
				nedge->setWidth(edge->getWidth());
				_modification._addededges.push_back(nedge);
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
				if (nedge->getWidth() > _modification.min_width) {
					_modification._addededges.push_back(nedge);
				}
				else {
					delete nedge;
				}
			}
		}
#pragma endregion

		_modification._addednodes.push_back(updnode);
		return 0;
	}

	/*Cut whole poligon from onde node*/
	int ModificationProcessor::CutPolyFromCurrentNode() {
		NavMeshNode* prev_node = nullptr;
		NavMeshEdge* first_edge = nullptr;
		for (int j = 0; j < _local_polygon.size(); j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(_modification.GetNextNodeID());
			int vert_count = 0;
			Vector2 j0vert = _local_polygon[j];
			Vector2 j1vert = _local_polygon[(j + 1) % _local_polygon.size()];
			Vector2 j2vert = _local_polygon[(j + 2) % _local_polygon.size()];
			bool node_side0 = !ModificationHelper::IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = ModificationHelper::IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					ModificationHelper::IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			vert_count += 4;
			updnode->_poly.vertCount = vert_count;
			updnode->_poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					ModificationHelper::IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
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
			if (edge->getWidth() > _modification.min_width) {
				_modification._addededges.push_back(edge);
				if (prev_node == nullptr) first_edge = edge;
				prev_node = updnode;
			}
			else {
				delete edge;
			}
			CopyVortexObstacles(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);
			CopyVortexEdges(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);

			_modification._addednodes.push_back(updnode);
#pragma endregion

		}
		first_edge->setNodes(prev_node, first_edge->getSecondNode());
		return _modification._addedobstacles.size();
	}

	int ModificationProcessor::CutCurveFromCurrentNode() {
		NavMeshNode* prev_node = nullptr;
		NavMeshEdge* first_edge = nullptr;
		for (int j = 0; j < _local_polygon.size() - 2; j++) {
			NavMeshNode* updnode = new NavMeshNode();
			updnode->setID(_modification.GetNextNodeID());
			int vert_count = 0;
			Vector2 j0vert = _local_polygon[j];
			Vector2 j1vert = _local_polygon[(j + 1) % _local_polygon.size()];
			Vector2 j2vert = _local_polygon[(j + 2) % _local_polygon.size()];
			bool node_side0 = !ModificationHelper::IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = ModificationHelper::IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					ModificationHelper::IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			if (j < _local_polygon.size() - 3) { vert_count++; }
			vert_count += 3;
			updnode->_poly.vertCount = vert_count;
			updnode->_poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;

			for (int i = 0; i < _current_node_poly->vertCount; i++) {
				auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
				if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					ModificationHelper::IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
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
			Vector2 crosspoint = crosspoints[j];
			if ((crosspoint - j1vert).Length() > _modification.min_width) {
				NavMeshEdge* edge = new NavMeshEdge();
				edge->setPoint(j1vert);
				edge->setWidth((crosspoint - j1vert).Length());
				edge->setDirection((crosspoint - j1vert) / (crosspoint - j1vert).Length());
				edge->setNodes(prev_node, updnode);
				if (edge->getWidth() > _modification.min_width) {
					_modification._addededges.push_back(edge);
					if (prev_node == nullptr) first_edge = edge;
					prev_node = updnode;
				}
				else {
					delete edge;
				}
			}
#pragma endregion

			CopyVortexObstacles(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);

			CopyVortexEdges(updnode, j, j0vert, j1vert, j2vert, node_side0, node_side1);
			_modification._addednodes.push_back(updnode);
		}

		//add node splited by v0 v1 line
#pragma region add_first_node
		NavMeshNode* updnode = new NavMeshNode();
		updnode->setID(_modification.GetNextNodeID());
		int vert_count = 0;
		Vector2 j0vert = _local_polygon[0];
		Vector2 j1vert = _local_polygon[1];
		Vector2 j2vert = _local_polygon[2];
		bool node_side0 = ModificationHelper::IsPointUnderLine(j0vert, j1vert, j2vert);
		//for correct vertex adding
		int post_index = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
			if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				vert_count++;
			}
		}
		vert_count += 3;
		updnode->_poly.vertCount = vert_count;
		updnode->_poly.vertIDs = new unsigned int[vert_count];


		int addedids = 0;
		for (int i = 0; i < _current_node_poly->vertCount; i++) {
			auto vertex = _modification._navmesh.vertices[_current_node_poly->vertIDs[i]];
			bool added = false;
			if (ModificationHelper::IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
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
		if (first_edge != nullptr) {
			first_edge->setNodes(updnode, first_edge->getSecondNode());
			_modification._addednodes.push_back(updnode);
		}
#pragma endregion

		return 0;
	}

	/*Adds crosspoints for polygon cut*/
	void ModificationProcessor::FillAddedVertices(bool isCurve) {
		int max = isCurve ? _local_polygon.size() - 2 : _local_polygon.size();
		crosspoints_ids = std::vector<unsigned int>(max);
		crosspoints = std::vector<Vector2>(max);
		for (int i = 0; i < max; i++) {
			Vector2 crosspoint = ModificationHelper::FindPolyAndSegmentCrosspoints(
				_local_polygon[i],
				_local_polygon[(i + 1) % _local_polygon.size()],
				_current_node_poly, true)[0];
			crosspoints_ids[i] = _modification.AddVertex(crosspoint);
			crosspoints[i] = crosspoint;
		}
	}

	void ModificationProcessor::CopyVortexObstacles(NavMeshNode* updnode, int j,
		Vector2 j0vert, Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict) {

		std::vector<NavMeshObstacle*> tmp_obst = std::vector<NavMeshObstacle*>();
		NavMeshObstacle *obst = new NavMeshObstacle();
		obst->setNode(updnode);
		Vector2 poly0 = _local_polygon[(j + 1) % _local_polygon.size()];
		Vector2 poly1 = _local_polygon[(j + 2) % _local_polygon.size()];
		obst->_point = poly0;
		obst->_unitDir = (poly1 - poly0) / (poly1 - poly0).Length();
		obst->_length = (poly1 - poly0).Length();
		//TODO
		//some times here spawns unnecessary obstacles then j = -1. if should be removed
		if (j != -1) {
			tmp_obst.push_back(obst);
		}

		for (int i = 0; i < _current_node->_obstCount; i++) {
			NavMeshObstacle *obst = _current_node->_obstacles[i];
			Vector2 p0 = obst->getP0();
			Vector2 p1 = obst->getP1();
			bool p0side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p0, node_side0, true) &&
				ModificationHelper::IsPointUnderLine(j1vert, j2vert, p0, node_side1);
			bool p1side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p1, node_side0, true) &&
				ModificationHelper::IsPointUnderLine(j1vert, j2vert, p1, node_side1);
			if (onestrict) {
				p0side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p0, node_side0, true);
				p1side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p1, node_side0, true);
			}

			if (p0side && p1side) {
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
					if (ModificationHelper::IsPointsOnLine(p0, p1, v0)) divpoint = v0;
					else if (ModificationHelper::IsPointsOnLine(p0, p1, v1)) divpoint = v1;
					else if (ModificationHelper::IsPointsOnLine(p0, p1, poly0)) divpoint = poly0;
					else if (ModificationHelper::IsPointsOnLine(p0, p1, poly1)) divpoint = poly1;
					//TODO is it valid?
					else divpoint = p0side ? p0 : p1;
					NavMeshObstacle *nobst = new NavMeshObstacle();
					nobst->setNode(updnode);
					nobst->_point = divpoint;
					if (p0side) {
						nobst->_unitDir = (p0 - divpoint) / (p0 - divpoint).Length();
						nobst->_length = (p0 - divpoint).Length();
					}
					else {
						nobst->_unitDir = (p1 - divpoint) / (p1 - divpoint).Length();
						nobst->_length = (p1 - divpoint).Length();
					}
					tmp_obst.push_back(nobst);
#pragma endregion
				}
				else {
#pragma region both_x_on_obst
					//is both crosspoints on edge?
					if (ModificationHelper::IsPointsOnLine(p0, p1, v0)
						&& ModificationHelper::IsPointsOnLine(p0, p1, v1)) {
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

		for (int i = tmp_obst.size() - 1; i >=0; i--) {
			if (tmp_obst[i]->length() < _modification.min_width) {
				delete tmp_obst[i];
				tmp_obst.erase(tmp_obst.begin() + i);
			}
		}

		updnode->_obstacles = new NavMeshObstacle*[tmp_obst.size()];
		updnode->_obstCount = tmp_obst.size();
		for (int i = 0; i < tmp_obst.size(); i++) {
			updnode->_obstacles[i] = tmp_obst[i];
			_modification._addedobstacles.push_back(tmp_obst[i]);
		}
	}


	void ModificationProcessor::CopyVortexEdges(NavMeshNode* updnode, int j,
		Vector2 j0vert, Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict) {
		std::vector<NavMeshEdge*> tmp_edges = std::vector<NavMeshEdge*>();

		for (int i = 0; i < _current_node->_edgeCount; i++) {
			NavMeshEdge *edge = _current_node->_edges[i];
			Vector2 p0 = edge->getP0();
			Vector2 p1 = edge->getP1();
			bool p0side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p0, node_side0, true) &&
				ModificationHelper::IsPointUnderLine(j1vert, j2vert, p0, node_side1);
			bool p1side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p1, node_side0, true) &&
				ModificationHelper::IsPointUnderLine(j1vert, j2vert, p1, node_side1);
			if (onestrict) {
				p0side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p0, node_side0, true);
				p1side = ModificationHelper::IsPointUnderLine(j0vert, j1vert, p1, node_side0, true);
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
					if (ModificationHelper::IsPointsOnLine(p0, p1, v0)) divpoint = v0;
					else if (ModificationHelper::IsPointsOnLine(p0, p1, v1)) divpoint = v1;
					//TODO is it valid?
					else divpoint = p0side ? p0 : p1;
					NavMeshEdge *nedge = new NavMeshEdge();
					nedge->setNodes(updnode, nullptr);
					nedge->setPoint(divpoint);
					if (p0side) {
						nedge->setDirection((p0 - divpoint) / (p0 - divpoint).Length());
						nedge->setWidth((p0 - divpoint).Length());
					}
					else {
						nedge->setDirection((p1 - divpoint) / (p1 - divpoint).Length());
						nedge->setWidth((p1 - divpoint).Length());
					}
					tmp_edges.push_back(nedge);
				}
				else {
					//is both crosspoints on edge?
					if (ModificationHelper::IsPointsOnLine(p0, p1, v0)
						&& ModificationHelper::IsPointsOnLine(p0, p1, v1)) {
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

		for (int i = tmp_edges.size() - 1; i >= 0; i--) {
			if (tmp_edges[i]->getWidth() < _modification.min_width ) {
				delete tmp_edges[i];
				tmp_edges.erase(tmp_edges.begin() + i);
			}
		}

		updnode->_edges = new NavMeshEdge*[tmp_edges.size()];
		updnode->_edgeCount = tmp_edges.size();
		for (int i = 0; i < tmp_edges.size(); i++) {
			updnode->_edges[i] = tmp_edges[i];
			_modification._addededges.push_back(tmp_edges[i]);
		}
	}


}
