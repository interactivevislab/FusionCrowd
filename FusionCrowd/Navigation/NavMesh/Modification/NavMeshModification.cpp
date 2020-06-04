#include "NavMeshModification.h"
#include "ModificationHelper.h"

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	NavMeshModification::NavMeshModification(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query)
		: _localizer(localizer), _navmesh(navmesh), _spatial_query(spatial_query)
	{
		_addedvertices = std::vector<Vector2>();
		_nodes_ids_to_delete = std::vector<size_t>();
		_addednodes = std::vector<NavMeshNode*>();
		_addededges = std::vector<NavMeshEdge*>();
		_addedobstacles = std::vector<NavMeshObstacle*>();
	}

	NavMeshModification::~NavMeshModification()
	{
	}

	//adds vertex and returns its ID
	unsigned int NavMeshModification::AddVertex(DirectX::SimpleMath::Vector2 v) {
		_addedvertices.push_back(v);
		return _navmesh.vCount + _global_polygon.size() + _addedvertices.size() - 1;
	}

	NavMeshNode* NavMeshModification::FindNode(size_t id) {
		for (int i = 0; i < _navmesh.nCount; i++) {
			if (_navmesh.nodes[i]._id == id) return &_navmesh.nodes[i];
		}
		for (auto n : _addednodes) {
			if (n->_id == id) return n;
		}
		return nullptr;
	}

	unsigned int NavMeshModification::GetNextNodeID() {
		return _navmesh.nCount + _addednodes.size();
	}

	NavMeshNode* NavMeshModification::GetGlobalPolygonNodeByVertexId(size_t id) {
		return _navmesh.GetNodeByID(_localizer->getNodeId(_global_polygon[id]));
	}

	/*--------------------------------------FINALIZE--------------------------------------------*/

	/*Adds all created nodes and vertexes*/
	/*
	Finalize
		RemoveInsidePoly
		RemoveObstaclesOnEdges
		Validate obstalces/edges by width
		Copy vertices
		Copy old edges
		Copy new obstalces
		Save edge obstalces ids
		Finalize nodes
		Load edge pbstacles ids
		Finalize edges
		Add new obstalces
		Add new edges
		Fill nodes obstalce/edges arrays
		Clear
	*/
	int NavMeshModification::Finalize() {
		RemoveNodesInsidePoly(_global_polygon);
		RemoveObstaclesOnEdges();

		for (int i = _addededges.size() - 1; i >= 0; i--) {
			if (_addededges[i]->getWidth() <= min_width) {
				delete _addededges[i];
				_addededges.erase(_addededges.begin() + i);
			}
		}
		for (int i = _addedobstacles.size() - 1; i >= 0; i--) {
			if (_addedobstacles[i]->_length <= min_width) {
				delete _addedobstacles[i];
				_addedobstacles.erase(_addedobstacles.begin() + i);
			}
		}

		FinalizeVertices();

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

#pragma region save_eo_dependencies
		std::vector<unsigned int> first_nodes_ids = std::vector<unsigned int>(vtmp_edges.size());
		std::vector<unsigned int> second_nodes_ids = std::vector<unsigned int>(vtmp_edges.size());
		for (int i = 0; i < vtmp_edges.size(); i++) {
			first_nodes_ids[i] = vtmp_edges[i].getFirstNode()->_id;
			second_nodes_ids[i] = vtmp_edges[i].getSecondNode()->_id;
		}

		std::vector<long int> fnid_added = std::vector<long int>(_addededges.size());
		std::vector<long int> snid_added = std::vector<long int>(_addededges.size());
		for (int i = 0; i < _addededges.size(); i++) {
			fnid_added[i] = _addededges[i]->getFirstNode() != nullptr ? _addededges[i]->getFirstNode()->_id : -1;
			snid_added[i] = _addededges[i]->getSecondNode() != nullptr ? _addededges[i]->getSecondNode()->_id : -1;
		}

		std::vector<unsigned int> obtacle_nodes_ids = std::vector<unsigned int>(tmp_obstacles.size());
		for (int i = 0; i < tmp_obstacles.size(); i++) {
			obtacle_nodes_ids[i] = tmp_obstacles[i].getNode()->_id;
			if (obtacle_nodes_ids[i] > _navmesh.nCount + _addednodes.size() - 1) throw obtacle_nodes_ids[i];
		}

		for (int i = _addedobstacles.size() - 1; i >= 0; i--) {
			if (_addedobstacles[i]->getNode()->_id > _navmesh.nCount + _addednodes.size() - 1) {
				_addedobstacles.erase(_addedobstacles.begin() + i);
				//TODO that happens then vertex on node edge
				continue;
				throw 1;
			}
		}

		std::vector<long int> oadded_nodes_ids = std::vector<long int>(_addedobstacles.size());
		for (int i = 0; i < _addedobstacles.size(); i++) {
			oadded_nodes_ids[i] = _addedobstacles[i]->getNode()->_id;
		}

#pragma endregion

		FinalizeNodes();
		_localizer->Update(_addednodes, _nodes_ids_to_delete);

#pragma region load_eo_dependencies
		for (int i = 0; i < vtmp_edges.size(); i++) {
			NavMeshNode* n0 = _navmesh.GetNodeByID(first_nodes_ids[i]);
			NavMeshNode* n1 = _navmesh.GetNodeByID(second_nodes_ids[i]);
			vtmp_edges[i].setNodes(n0, n1);
		}

		for (int i = 0; i < _addededges.size(); i++) {
			NavMeshNode* n0 = fnid_added[i] != -1 ? _navmesh.GetNodeByID(fnid_added[i]) : nullptr;
			NavMeshNode* n1 = snid_added[i] != -1 ? _navmesh.GetNodeByID(snid_added[i]) : nullptr;
			_addededges[i]->setNodes(n0, n1);
		}

		for (int i = 0; i < tmp_obstacles.size(); i++) {
			NavMeshNode* n = _navmesh.GetNodeByID(obtacle_nodes_ids[i]);
			tmp_obstacles[i].setNode(n);
		}

		for (int i = 0; i < _addedobstacles.size(); i++) {
			NavMeshNode* n = _navmesh.GetNodeByID(oadded_nodes_ids[i]);
			_addedobstacles[i]->setNode(n);
		}

#pragma endregion

		FinalizeEdges();

#pragma region add_new_obsts

		for (int i = 0; i < _addedobstacles.size(); i++) {
			if (ValidateObstalce(_addedobstacles[i])) {
				_addedobstacles[i]->_id = oid;
				oid++;
				tmp_obstacles.push_back(*_addedobstacles[i]);
			}
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

		FillNodeEdgeObstaclesArrays();

		for (int i = 0; i < _addednodes.size(); i++) {
			delete _addednodes[i];
		}

		_spatial_query->Update();
		_navmesh.IncVersion();

		Clear();
		return 0;
	}


	void NavMeshModification::RemoveObstaclesOnEdges() {
		for (int i = _addedobstacles.size() - 1; i >= 0; i--) {
			auto obst = _addedobstacles[i];
			bool delete_obst = false;
			for (auto e : _addededges) {
				if ((Vector2::Distance(e->getP0(), obst->getP0()) < 1e-3f &&
					Vector2::Distance(e->getP1(), obst->getP1()) < 1e-3f) ||
					(Vector2::Distance(e->getP0(), obst->getP1()) < 1e-3f &&
						Vector2::Distance(e->getP1(), obst->getP0()) < 1e-3f)) {
					delete_obst = true;
					break;
				}
			}
			if (delete_obst) {
				_addedobstacles.erase(_addedobstacles.begin() + i);
				delete obst;
				continue;
			}
			for (int j = 0; j < _navmesh.eCount; j++) {
				auto& e = _navmesh.edges[j];
				if ((Vector2::Distance(e.getP0(), obst->getP0()) < 1e-3f &&
					Vector2::Distance(e.getP1(), obst->getP1()) < 1e-3f) ||
					(Vector2::Distance(e.getP0(), obst->getP1()) < 1e-3f &&
						Vector2::Distance(e.getP1(), obst->getP0()) < 1e-3f)) {
					delete_obst = true;
					break;
				}
			}
			if (delete_obst) {
				_addedobstacles.erase(_addedobstacles.begin() + i);
				delete obst;
			}
		}
	}

	/*Find second/first node if it == nullptr, adds obstacle if node can't be finded*/
	bool NavMeshModification::ProcessEdge(NavMeshEdge* edge) {
		Vector2 mid_edge = edge->getP0(edge->getWidth() / 2.0);
		if (edge->getFirstNode() != nullptr && edge->getSecondNode() != nullptr) {

			return true;
		}

		if (edge->getFirstNode() == nullptr && edge->getSecondNode() == nullptr) {
			//TODO that happens then vertex on node edge
			return false;
			throw 1;
		}
		float d = 0.25;
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

		if (node_id == exist_id) {
			//todo it shouldn't happend
			return false;
			throw 1;
		}
		if (node_id == NavMeshLocation::NO_NODE) {

			//TODO why it's happend?
			Vector2 perpendicular = Vector2(-edge->getDirection().y, edge->getDirection().x);
			perpendicular *= 0.25f;
			size_t n0 = _localizer->getNodeId(mid_edge + perpendicular);
			size_t n1 = _localizer->getNodeId(mid_edge - perpendicular);
			if (n0 != exist_id && n1 != exist_id) return false;

			NavMeshObstacle* obst = new NavMeshObstacle();
			obst->setNode(edge->getFirstNode() == nullptr ? edge->getSecondNode() : edge->getFirstNode());
			obst->_point = edge->getP0();
			obst->_unitDir = edge->getDirection();
			obst->_length = edge->getWidth();

			if (obst->_length > min_width) {
				_addedobstacles.push_back(obst);
			}
			else {
				delete obst;
			}
			return false;
		}
		if (edge->getFirstNode() == nullptr) {
			edge->setNodes(FindNode(node_id), edge->getSecondNode());
		}
		else {
			edge->setNodes(edge->getFirstNode(), FindNode(node_id));
		}
		return true;
	}


	bool NavMeshModification::ValidateObstalce(NavMeshObstacle* obst) {
		auto& poly = obst->getNode()->_poly;
		auto p0 = obst->getP0();
		auto p1 = obst->getP1();
		bool b0 = false, b1 = false;
		for (int i = 0; i < poly.vertCount; i++) {
			auto vpoly = poly.getVertexByPos(i);
			if (Vector2::Distance(vpoly, p0) < 1e-4f) b0 = true;
			if (Vector2::Distance(vpoly, p1) < 1e-4f) b1 = true;
			if (b0 && b1) return true;
		}
		return b0 && b1;
	}

	void NavMeshModification::FinalizeNodes() {
		for (auto n : _addednodes) {
			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(*n);
			ModificationHelper::ResetNodePolySequence(*n);
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

	void NavMeshModification::FillNodeEdgeObstaclesArrays() {

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
	}

	void NavMeshModification::FinalizeEdges() {
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
	}

	void NavMeshModification::FinalizeVertices() {
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
	}

	void NavMeshModification::RemoveNodesInsidePoly(std::vector<Vector2> poly) {
		float minx = INFINITY;
		float maxx = -INFINITY;
		float miny = INFINITY;
		float maxy = -INFINITY;

		for (auto v : poly) {
			if (v.x > maxx) maxx = v.x;
			if (v.x < minx) minx = v.x;
			if (v.y > maxy) maxy = v.y;
			if (v.y < miny) miny = v.y;
		}

		auto crossing_nodes_ids = _localizer->findNodesCrossingBB(BoundingBox(minx, miny, maxx, maxy));
		for (int j = 0; j < _navmesh.nCount; j++) {
			if (_navmesh.nodes[j].deleted) continue;
			if (std::find(crossing_nodes_ids.begin(),
				crossing_nodes_ids.end(),
				_navmesh.nodes[j]._id) != crossing_nodes_ids.end()) {
				auto& node_poly = _navmesh.nodes[j]._poly;
				bool node_inside = true;;
				for (int i = 0; i < node_poly.vertCount; i++) {
					auto node_v = node_poly.getVertexByPos(i);
					bool inside = false;
					for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
						float xi = poly[i].x, yi = poly[i].y;
						float xj = poly[j].x, yj = poly[j].y;

						bool intersect = ((yi > node_v.y) != (yj > node_v.y))
							&& (node_v.x < (xj - xi) * (node_v.y - yi) / (yj - yi) + xi);
						if (intersect) inside = !inside;
					}
					if (!inside) {
						node_inside = false;
						break;
					}
				}
				if (node_inside) _nodes_ids_to_delete.push_back(_navmesh.nodes[j]._id);
			}
		}
	}

	void NavMeshModification::Clear() {
		_addednodes.clear();
		_addededges.clear();
		_addedobstacles.clear();
		_nodes_ids_to_delete.clear();
		_addedvertices.clear();
		_global_polygon.clear();
	}
}
