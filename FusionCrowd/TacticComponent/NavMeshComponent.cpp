#include "NavMeshComponent.h"

#include "Path/PortalPath.h"
#include "Path/PathPlanner.h"
#include "Path/Route.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NavSystem.h"
#include "StrategyComponent/Goal/Goal.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Math/consts.h"
#include <iostream>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshComponent::NavMeshComponent(std::shared_ptr<Simulator> simulator,
		std::shared_ptr<NavMeshLocalizer> localizer, std::shared_ptr<NavMeshSpatialQuery> spatial_query) :
		_simulator(simulator), _localizer(localizer), _navMesh(localizer->getNavMesh()), _headingDevCos(cos(MathUtil::PI)), _spatial_query(spatial_query)
	{
	}

	NavMeshLocation NavMeshComponent::Replan(Vector2 fromPoint, const Goal & target, float agentRadius)
	{
		unsigned int from = GetClosestAvailableNode(fromPoint);
		unsigned int to = GetClosestAvailableNode(target.getCentroid());

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
		agentInfo.pos = GetClosestAvailablePoint(agentInfo.pos);

		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.location = Replan(agentInfo.pos, agentGoal, agentInfo.radius);

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

			if(groupId != Group::NO_GROUP)
			{
				SetGroupPrefVelocity(info, agtStruct, groupId, timeStep);
				continue;
			}

			if(IsReplanNeeded(info, agtStruct))
			{
				auto & curGoal = agtStruct.location.getPath()->getGoal();
				agtStruct.location = Replan(info.pos, curGoal, info.radius);
			}

			UpdateLocation(info, agtStruct, false);
			SetPrefVelocity(info, agtStruct, timeStep);
		}
	}

	Vector2 NavMeshComponent::GetClosestAvailablePoint(Vector2 p)
	{
		if (_localizer->findNodeBlind(p) != NavMeshLocation::NO_NODE)
		{
			return p;
		}

		float min_dist = INFINITY;
		Vector2 res;
		for (int i = _localizer->getNavMesh()->getNodeCount() - 1; i >= 0; i--)
		{
			if (!_localizer->getNavMesh()->GetNodeByPos(i).deleted)
			{
				Vector2 center = _localizer->getNavMesh()->GetNodeByPos(i).getCenter();
				if ((p - center).LengthSquared() < min_dist)
				{
					min_dist = (p - center).LengthSquared();
					res = center;
				}
			}
		}
		if (min_dist == INFINITY)
		{
			throw 1;
		}
		return res;
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
		for (int i = _localizer->getNavMesh()->getNodeCount() - 1; i >= 0; i--)
		{
			if (!_localizer->getNavMesh()->GetNodeByPos(i).deleted)
			{
				Vector2 center = _localizer->getNavMesh()->GetNodeByPos(i).getCenter();
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

	void NavMeshComponent::SetGroupPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, size_t groupId, float timeStep)
	{
		auto & group = _simulator->GetGroup(groupId);
		auto & groupDummy = _simulator->GetSpatialInfo(group.dummyAgentId);

		float rot = atan2f(groupDummy.orient.x, groupDummy.orient.y);

		rot = rot - (MathUtil::PI / 4.0f);
		Vector2 relativePos = MathUtil::rotate(group.GetShape()->GetRelativePos(agentInfo.id), rot);
		Vector2 targetPos = groupDummy.pos + relativePos;
		Vector2 dir = targetPos - agentInfo.pos;

		float speed = dir.LengthSquared() / timeStep;
		if(agentInfo.maxSpeed < speed)
		{
			speed = agentInfo.maxSpeed;
		}
		agentInfo.prefVelocity.setSpeed(speed);

		dir.Normalize();
		agentInfo.prefVelocity.setSingle(dir);
	}

	void NavMeshComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentInfo.id);
		auto path = agentStruct.location.getPath();
		//TODO: replace with correct disk goal
		if ((agentGoal.getCentroid() - agentInfo.pos).Length() < 1.5f) {
			agentInfo.prefVelocity.setSpeed(0);
			return;
		}
		if (path == nullptr || path->getGoal().getID() != agentGoal.getID())
		{
			Vector2 goalPoint = agentGoal.getCentroid();
			unsigned int goalNode = _localizer->getNodeId(goalPoint);
			unsigned int agentNode = _localizer->getNodeId(agentInfo.pos);
			if (goalNode == NavMeshLocation::NO_NODE) {
				goalPoint = GetClosestAvailablePoint(agentGoal.getCentroid());
				_simulator->SetAgentGoal(agentInfo.id, goalPoint);
				goalNode = _localizer->getNodeId(goalPoint);
			}
			if (goalNode == NavMeshLocation::NO_NODE || agentNode == NavMeshLocation::NO_NODE)
			{
				agentInfo.prefVelocity.setSpeed(0);
				return;
			}
			agentStruct.location.setNode(agentNode);
			unsigned int agtNode = agentStruct.location.getNode();

			PortalRoute* route = _localizer->getPlanner()->getRoute(agtNode, goalNode, agentInfo.radius * 2.f);

			auto newPath = std::make_shared<PortalPath>(agentInfo.pos, agentGoal, route, agentInfo.radius);
			// assign it to the localizer
			agentStruct.location.setPath(newPath);
		}

		float dist = Vector2::Distance(agentGoal.getCentroid(), agentInfo.pos);

		if(dist < agentInfo.prefSpeed * timeStep)
		{
			agentInfo.prefVelocity.setSpeed(dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		}

		agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		path->setPreferredDirection(agentInfo, _headingDevCos);
	}

	unsigned int NavMeshComponent::UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const
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

	std::shared_ptr<NavMeshLocalizer> NavMeshComponent::GetLocalizer() const
	{
		return _localizer;
	}

	std::shared_ptr<NavMesh> NavMeshComponent::GetNavMesh() const
	{
		return _localizer->getNavMesh();
	}
}
