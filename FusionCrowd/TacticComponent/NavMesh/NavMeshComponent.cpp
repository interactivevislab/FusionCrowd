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

	NavMeshLocation NavMeshComponent::Replan(Vector2 fromPoint, const Goal & target, float agentRadius, bool& foundPath, size_t agentId)
	{
		size_t from = GetClosestAvailableNode(fromPoint);
		size_t to = GetClosestAvailableNode(target.getCentroid());

		auto planner = _localizer->getPlanner();
		auto* route = planner->getRoute(from, to, agentRadius, foundPath);

		std::vector<size_t> prePortalCount;
		std::vector<size_t> postPortalCount;
		std::vector<size_t> portalIndex;

		if (!foundPath)
		{
			for (size_t i = 0; i < _navMesh->teleportals.size(); i++)
			{
				//_simulator->SetAgentGoal(info.id, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));

				//auto& newCurGoal = _simulator->GetAgentGoal(info.id);

				//agtStruct.location = Replan(info.GetPos(), _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation), info.radius, pathFound);
				size_t newTo = GetClosestAvailableNode(_simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation).getCentroid());
				route = planner->getRoute(from, newTo, agentRadius, foundPath);
				if (foundPath)
				{
					prePortalCount.push_back(route->getPortalCount());
					portalIndex.push_back(i);
					//_simulator->SetAgentGoal(info.id, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));

					//auto& newCurGoal = _simulator->GetAgentGoal(info.id);

					//agtStruct.location = Replan(_navMesh->teleportals[i]._teleportLocation, newCurGoal, info.radius, pathFound);
					size_t newFrom = GetClosestAvailableNode(_navMesh->teleportals[i]._teleportLocation);
					route = planner->getRoute(newFrom, to, agentRadius, foundPath);
					if (foundPath)
					{
						postPortalCount.push_back(route->getPortalCount());
						//_simulator->SetAgentGoal(agentId, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));
						//break;
					}
					else
					{
						prePortalCount.pop_back();
						portalIndex.pop_back();
						continue;
					}
					//break;
				}
			}
			if (prePortalCount.size()>0)
			{
				size_t lowestCount = prePortalCount[0] + postPortalCount[0];
				size_t shortestPortalIndex = portalIndex[0];

				for (size_t i = 1; i < prePortalCount.size(); i++)
				{
					if (lowestCount > prePortalCount[i] + postPortalCount[i])
					{
						lowestCount = prePortalCount[i] + postPortalCount[i];
						shortestPortalIndex = portalIndex[i];
					}
				}

				_simulator->SetAgentGoal(agentId, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[shortestPortalIndex]._portalLocation));
			}
		}

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

		bool pathFound = false;
		agtStruct.location = Replan(agentInfo.GetPos(), agentGoal, agentInfo.radius, pathFound, agentInfo.id);

		_agents.insert(std::pair<size_t, AgentStruct>(id, agtStruct));
	}

	bool NavMeshComponent::DeleteAgent(size_t id)
	{
		_agents.erase(id);
		return true;
	}

	void NavMeshComponent::Update(float timeStep)
	{
		for (auto & agentData : _agents)
		{
			auto& agtStruct = agentData.second;
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
				bool pathFound = false;
				agtStruct.location = Replan(info.GetPos(), curGoal, info.radius, pathFound, info.id);

				//if (!pathFound)
				//{
				//	for (size_t i = 0; i < _navMesh->teleportals.size(); i++)
				//	{
				//		//_simulator->SetAgentGoal(info.id, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));

				//		auto & newCurGoal = _simulator->GetAgentGoal(info.id);

				//		agtStruct.location = Replan(info.GetPos(), _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation), info.radius, pathFound);
				//		if (pathFound)
				//		{
				//			//_simulator->SetAgentGoal(info.id, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));

				//			auto& newCurGoal = _simulator->GetAgentGoal(info.id);

				//			agtStruct.location = Replan(_navMesh->teleportals[i]._teleportLocation, newCurGoal, info.radius, pathFound);
				//			if (pathFound)
				//			{
				//				_simulator->SetAgentGoal(info.id, _simulator->GetGoalFactory().CreatePointGoal(_navMesh->teleportals[i]._portalLocation));
				//				break;
				//			}
				//			else
				//			{
				//				continue;
				//			}
				//			//break;
				//		}
				//	}
				//}
			}

			UpdateLocation(info, agtStruct, false);
			SetPrefVelocity(info, agtStruct, timeStep);
			
			info.zPos = _localizer->getNavMesh()->GetNodeByPos(getNodeId(info.id)).getElevation(info.GetPos());
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
		return _agents.find(agentId)->second.location.getNode();
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
