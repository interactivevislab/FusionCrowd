#include "NavMeshComponent.h"

#include "Path/PortalPath.h"
#include "Path/PathPlanner.h"
#include "Path/Route.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NavSystem.h"
#include "StrategyComponent/Goal/Goal.h"
#include "TacticComponent/PrefVelocity.h"
#include "Math/consts.h"

#include <iostream>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshComponent::NavMeshComponent(std::shared_ptr<Simulator> simulator,
		std::shared_ptr<NavMeshLocalizer> localizer, std::shared_ptr<NavMeshSpatialQuery> spatial_query) :
		_simulator(simulator), _localizer(localizer), _navMesh(localizer->getNavMesh()), _headingDevCos(cos(Math::PI)), _spatial_query(spatial_query)
	{
	}

	NavMeshLocation NavMeshComponent::Replan(Vector2 fromPoint, const Goal & target, float agentRadius)
	{
		size_t from = GetClosestAvailableNode(fromPoint);
		size_t to = GetClosestAvailableNode(target.getCentroid());

		auto planner = _localizer->getPlanner();
		auto* route = planner->getRoute(from, to, agentRadius);
		std::shared_ptr<PortalPath> path = std::make_shared<PortalPath>(fromPoint, target, route, agentRadius);

		NavMeshLocation location(from);
		location.setPath(path);

		return location;
	}

	void NavMeshComponent::AddAgent(size_t id)
	{
		auto & agentGoal = _simulator->GetAgentGoal(id);
		AgentSpatialInfo & agentInfo = _simulator->GetSpatialInfo(id);
		agentInfo.SetPos(GetClosestAvailablePoint(agentInfo.GetPos()));

		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.location = Replan(agentInfo.GetPos(), agentGoal, agentInfo.radius);

		_agents.push_back(agtStruct);
	}

	bool NavMeshComponent::DeleteAgent(size_t id)
	{
		for (int i = 0; i < _agents.size(); i++) {
			if (_agents[i].id == id) {
				_agents.erase(_agents.begin() + i);
				break;
			}
		}
		return false;
	}

	void NavMeshComponent::Update(float timeStep)
	{
		for (auto & agtStruct : _agents)
		{
			size_t id = agtStruct.id;

			size_t groupId = _simulator->GetAgent(id).GetGroupId();
			AgentSpatialInfo & info = _simulator->GetSpatialInfo(id);

			if(groupId != IGroup::NO_GROUP)
			{
				auto grp = _simulator->GetGroup(groupId);
				auto & dummy = _simulator->GetSpatialInfo(grp->GetDummyId());
				if (grp == nullptr) {
					info.prefVelocity.setSpeed(0);
					continue;
				}

				grp->SetAgentPrefVelocity(dummy, info, timeStep);
				continue;
			}

			if(IsReplanNeeded(info, agtStruct))
			{
				auto & curGoal = _simulator->GetAgentGoal(info.id);
				agtStruct.location = Replan(info.GetPos(), curGoal, info.radius);
			}

			UpdateLocation(info, agtStruct, false);
			SetPrefVelocity(info, agtStruct, timeStep);
		}
	}

	Vector2 NavMeshComponent::GetClosestAvailablePoint(Vector2 p)
	{
		return _localizer->GetClosestAvailablePoint(p);
	}

	size_t NavMeshComponent::GetClosestAvailableNode(Vector2 p)
	{
		auto correct = _localizer->findNodeBlind(p);
		if (correct != NavMeshLocation::NO_NODE)
		{
			return correct;
		}

		float min_dist = INFINITY;
		size_t res;
		auto navMesh = _localizer->getNavMesh();
		for (size_t i = 0; i < navMesh->getNodeCount(); i++)
		{
			const auto & node = navMesh->GetNodeByPos(i);
			if (!node.deleted)
			{
				Vector2 center = node.getCenter();
				if ((p - center).LengthSquared() < min_dist)
				{
					min_dist = (p - center).LengthSquared();
					res = i;
				}
			}
		}
		if (min_dist == INFINITY)
		{
			throw 1;
		}

		return res;
	}

	bool NavMeshComponent::IsReplanNeeded(AgentSpatialInfo& agentInfo, AgentStruct& agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentInfo.id);
		auto path = agentStruct.location.getPath();
		return
			path == nullptr ||
			path->getGoal().getID() != agentGoal.getID() ||
			!path->IsValid(_navMesh->GetVersion());
	}

	void NavMeshComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentInfo.id);
		auto path = agentStruct.location.getPath();

		if (agentGoal.getGeometry()->containsPoint(agentInfo.GetPos()))
		{
			agentInfo.prefVelocity.setSpeed(0);
			return;
		}

		path->setPrefVelocity(agentInfo, _headingDevCos, timeStep);
	}

	unsigned int NavMeshComponent::UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const
	{
		NavMeshLocation & loc = agentStruct.location;
		if (loc._hasPath)
		{
			return loc._path->updateLocation(agentInfo, _navMesh, _localizer, _localizer->getPlanner());
		}

		unsigned int oldLoc = loc.getNode();
		unsigned int newLoc = oldLoc;

		const Vector2 & p = agentInfo.GetPos();
		unsigned int oldNode = (unsigned int)loc._nodeID;
		if (loc._nodeID == NavMeshLocation::NO_NODE)
		{
			loc._nodeID = _localizer->findNodeBlind(p);
		}
		else
		{
			const NavMeshNode& node = _navMesh->GetNodeByPos((unsigned int)loc._nodeID);
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

		return newLoc;
	}

	size_t NavMeshComponent::getNodeId(size_t agentId) const
	{
		unsigned int node = NavMeshLocation::NO_NODE;
		return _agents[agentId].location.getNode();
	}

	std::shared_ptr<NavMeshLocalizer> NavMeshComponent::GetLocalizer() const
	{
		return _localizer;
	}

	std::shared_ptr<NavMesh> NavMeshComponent::GetNavMesh() const
	{
		return _localizer->getNavMesh();
	}
}
