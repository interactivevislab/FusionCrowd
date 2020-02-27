#include "EdgeObstacleReplaner.h"


#include "NavMeshModification.h"
#include "ModificationHelper.h"

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	EdgeObstacleReplaner::EdgeObstacleReplaner(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer)
		: _navmesh(navmesh), _localizer(localizer)
	{
	}


	EdgeObstacleReplaner::~EdgeObstacleReplaner()
	{
	}

	void EdgeObstacleReplaner::Replan() {

		for (int i = 0; i < _navmesh.nCount; i++) {
			ProcessNode(_navmesh.nodes[i]);
		}
		for (int i = _edges.size() - 1; i >= 0; i--) {
			if (!ValidateEdge(_edges[i])) {
				delete _edges[i];
				_edges.erase(_edges.begin() + i);
			}
		}
		FillNavmesh();
	}


	void EdgeObstacleReplaner::ProcessNode(NavMeshNode& node) {
		auto& poly = node._poly;
		for (int i = 0; i < poly.vertCount; i++) {
			Vector2 v0 = poly.getVertexByPos(i);
			Vector2 v1 = poly.getVertexByPos((i + 1) % poly.vertCount);
			Vector2 ort = v1 - v0;
			ort = Vector2(-ort.y, ort.x);
			ort.Normalize();
			ort *= DELTA;
			Vector2 tmp = v1 - v0;
			tmp /= 2.0f;
			Vector2 mid = v0 + tmp;
			Vector2 checkpoint;
			if (Vector2::Distance(mid + ort, node.getCenter()) < Vector2::Distance(mid - ort, node.getCenter())) {
				checkpoint = mid - ort;
			}
			else {
				checkpoint = mid + ort;
			}
			auto node_id = _localizer->getNodeId(checkpoint);
			auto dir = v1 - v0;
			dir.Normalize();
			if (NavMeshLocation::NO_NODE == node_id) {
				NavMeshObstacle* obst = new NavMeshObstacle();
				obst->setNode(&node);
				obst->_point = v0;
				obst->_unitDir = dir;
				obst->_length = (v1 - v0).Length();
				_obstacles.push_back(obst);
			}
			else {
				NavMeshEdge* edge = new NavMeshEdge();
				edge->setPoint(v0);
				edge->setDirection(dir);
				edge->setNodes(&node, _navmesh.GetNodeByID(node_id));
				edge->setWidth((v1 - v0).Length());
				_edges.push_back(edge);
			}
		}
	}


	bool EdgeObstacleReplaner::ValidateEdge(NavMeshEdge* edge) {
		for (auto e : _edges) {
			if (e == edge) continue;
			if ((e->getFirstNode() == edge->getFirstNode() && e->getSecondNode() == edge->getSecondNode()) ||
				(e->getFirstNode() == edge->getSecondNode() && e->getSecondNode() == edge->getFirstNode())) {
				if (edge->getWidth() > e->getWidth()) return false;
			}
		}
		return true;
	}

	void EdgeObstacleReplaner::FillNavmesh() {
		//navmesh obst
		_navmesh.obstacles.clear();
		_navmesh.obstCount = _obstacles.size();
		for (int i = 0; i < _obstacles.size(); i++) {
			_navmesh.obstacles.push_back(*_obstacles[i]);
		}

		//navmesh edges
		delete[] _navmesh.edges;
		_navmesh.eCount = _edges.size();
		_navmesh.edges = new NavMeshEdge[_edges.size()];
		for (int i = 0; i < _edges.size(); i++) {
			_navmesh.edges[i] = *_edges[i];
		}

		//nodes obstacles
		for (int i = 0; i < _navmesh.nCount; i++) {
			auto& node = _navmesh.nodes[i];
			std::vector<NavMeshObstacle*> node_obst;
			for (int j = 0; j < _navmesh.obstCount; j++) {
				auto o = &_navmesh.obstacles[j];
				if (o->getNode()->_id == node._id) {
					node_obst.push_back(o);
				}
			}
			node._obstCount = node_obst.size();
			node._obstacles = new NavMeshObstacle*[node_obst.size()];
			for (int j = 0; j < node_obst.size(); j++) {
				node._obstacles[j] = node_obst[j];
			}
		}

		//nodes edges
		for (int i = 0; i < _navmesh.nCount; i++) {
			auto& node = _navmesh.nodes[i];
			std::vector<NavMeshEdge*> node_edge;
			for (int j = 0; j < _navmesh.eCount; j++) {
				auto e = &_navmesh.edges[j];
				if (e->getFirstNode()->_id == node._id || e->getSecondNode()->_id == node._id) {
					node_edge.push_back(e);
				}
			}
			node._edgeCount = node_edge.size();
			node._edges = new NavMeshEdge*[node_edge.size()];
			for (int j = 0; j < node_edge.size(); j++) {
				node._edges[j] = node_edge[j];
			}
		}

	}
}
