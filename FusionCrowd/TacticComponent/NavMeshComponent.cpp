#include "NavMeshComponent.h"
#include "Path/PortalPath.h"
#include "Path/PathPlanner.h"
#include "Path/Route.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NavSystem.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshComponent::NavMeshComponent(Simulator & simulator, const char* navMeshPath) : _simulator(simulator)
	{
		_localizer = std::make_shared<NavMeshLocalizer>(navMeshPath, true);
		_navMesh = _localizer->getNavMesh();
	}

	void NavMeshComponent::AddAgent(size_t id)
	{
		auto agentGoal = _simulator.GetAgentGoal(id);
		AgentSpatialInfo & agentInfo = _simulator.GetNavSystem().GetSpatialInfo(id);

		unsigned int from = _localizer->getNodeId(agentInfo.pos);
		unsigned int to = _localizer->getNodeId(agentGoal->getCentroid());

		assert(from != NavMeshLocation::NO_NODE && "Agent is not on the nav mesh");
		assert(to != NavMeshLocation::NO_NODE && "Agent goal is not on the nav mesh");

		std::shared_ptr<PathPlanner> planner = _localizer->getPlanner();
		PortalRoute * route = planner->getRoute(from, to, agentInfo.radius);
		auto path = std::make_shared<PortalPath>(agentInfo.pos, agentGoal, route, agentInfo.radius);

		NavMeshLocation location(_localizer->getNodeId(agentInfo.pos));
		location.setPath(path);

		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.location = location;

		_agents.push_back(agtStruct);
	}

	bool NavMeshComponent::RemoveAgent(size_t id)
	{
		return false;
	}

	void NavMeshComponent::Update(float timeStep)
	{
		for (auto agtStruct : _agents)
		{
			size_t id = agtStruct.id;
			AgentSpatialInfo & info = _simulator.GetNavSystem().GetSpatialInfo(id);

			updateLocation(info, agtStruct, false);
			setPrefVelocity(info, agtStruct);
		}
	}

	void NavMeshComponent::UpdateNavMesh(DirectX::SimpleMath::Vector2 point)
	{
		int nCount = _navMesh->getNodeCount();
		NavMeshEdge** _edges = nullptr;
		int edgeCount;
		for (int i = 0; i < nCount; i++)
		{
			FusionCrowd::NavMeshNode& node = _navMesh->GetNode(i);
			DirectX::SimpleMath::Vector2 c = node.getCenter();
			if (c == point)
			{
				edgeCount = node.getEdgeCount();
				_edges = new NavMeshEdge*[edgeCount];
				for (int j = 0; j < edgeCount; j++)
				{
					_edges[j] = node.getEdge(j);
					int eCount = node.getEdgeCount();
				}
			}
		}
		NavMeshEdge** _edgesNew = nullptr;
		for (int i = 0; i < nCount; i++)
		{
			FusionCrowd::NavMeshNode& node = _navMesh->GetNode(i);
			int eCount = node.getEdgeCount();
			std::vector<int> t;
			for (int j = 0; j < eCount; j++)
			{
				for (int e = 0; e < edgeCount; e++)
				{
					if (node.getEdge(j) == _edges[e])
					{
						t.push_back(j);
					}
				}
			}
			if (t.size() != 0)
			{
				_edgesNew = new NavMeshEdge*[eCount - t.size()];
				int e = 0;
				for (int j = 0; j < eCount; j++)
				{
					if (std::find(t.begin(), t.end(), j) == t.end())
					{
						_edgesNew[e] = node.getEdge(j);
						e++;
					}
				}
				node._edgeCount = eCount - t.size();
				if (node._edgeCount == 0)
				{
					_edgesNew = nullptr;
				}
				node._edges = _edgesNew;
			}
		}
	}

	void NavMeshComponent::setPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct)
	{
		auto agentGoal = _simulator.GetAgentGoal(agentInfo.id);
		auto path = agentStruct.location.getPath();
		if (path == nullptr || path->getGoal() != agentGoal)
		{
			Vector2 goalPoint = agentGoal->getCentroid();
			unsigned int goalNode = _localizer->getNodeId(goalPoint);
			if (goalNode == NavMeshLocation::NO_NODE)
			{
				return;
			}

			unsigned int agtNode = agentStruct.location.getNode();

			PortalRoute* route = _localizer->getPlanner()->getRoute(agtNode, goalNode, agentInfo.radius * 2.f);

			path = std::make_shared<PortalPath>(agentInfo.pos, agentGoal, route, agentInfo.radius);
			// assign it to the localizer
			agentStruct.location.setPath(path);
		}
		agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		path->setPreferredDirection(agentInfo, _headingDevCos);
	}

	unsigned int NavMeshComponent::updateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const
	{
		// NOTE: This will create a default location instance if the agent didn't already
		//	have one
		NavMeshLocation & loc = agentStruct.location;

		unsigned int oldLoc = loc.getNode();
		unsigned int newLoc = oldLoc;
		if (loc._hasPath)
		{
			newLoc = loc._path->updateLocation(agentInfo, _navMesh, _localizer, _localizer->getPlanner());
		}
		else
		{
			//if ( _trackAll || force ) {
			const Vector2 & p = agentInfo.pos;
			unsigned int oldNode = (unsigned int)loc._nodeID;
			if (loc._nodeID == NavMeshLocation::NO_NODE)
			{
				loc._nodeID = _localizer->findNodeBlind(p);
			}
			else
			{
				const NavMeshNode& node = _navMesh->GetNode((unsigned int)loc._nodeID);
				if (!node.containsPoint(p))
				{
					// not in current node
					loc._nodeID = _localizer->testNeighbors(node, p);
					if (loc._nodeID == NavMeshLocation::NO_NODE)
					{
						loc._nodeID = _localizer->findNodeBlind(p);
					}
				}
			}
			if (loc._nodeID == NavMeshLocation::NO_NODE)
			{
				loc._nodeID = oldNode;
			}
			newLoc = (unsigned int)loc._nodeID;
		}

		_localizer->updateAgentPosition(agentInfo.id, oldLoc, newLoc);

		return newLoc;
	}

	unsigned int NavMeshComponent::getNodeId(size_t agentId) const
	{
		unsigned int node = NavMeshLocation::NO_NODE;
		return _agents[agentId].location.getNode();
	}

	unsigned int NavMeshComponent::getNodeId(size_t agentId, const std::string& grpName, bool searchAll)
	{
		unsigned int nodeId = getNodeId(agentId);
		if (nodeId == NavMeshLocation::NO_NODE)
		{
			//node = findNodeInGroup(agent->_pos, grpName, searchAll);
			if (nodeId != NavMeshLocation::NO_NODE)
			{
				_agents[agentId].location.setNode(nodeId);
			}
		}
		return nodeId;
	}

	std::shared_ptr<NavMeshLocalizer> NavMeshComponent::GetLocalizer()
	{
		return _localizer;
	}

	/*
	void NavMeshComponent::setNode(size_t agentID, unsigned int nodeID)
	{
		_locations[agentID].setNode(nodeID);
	}

	PortalPath * NavMeshComponent::getPath(size_t id)
	{
		PortalPath * path = 0x0;
		auto loc = _agents[id].location;
		if (loc.isPath()) {
			path = loc._path;
		}
		return path;
	}

	void NavMeshComponent::setPath(size_t agentID, PortalPath * path)
	{
		_locations[agentID].setPath(path);
	}

	void NavMeshComponent::clearPath(size_t agentID)
	{
		if (_locations.count(agentID) > 0) {
			_locations[agentID].clearPath();
		}
	}*/

	NavMeshComponent::~NavMeshComponent()
	{
	}
}
