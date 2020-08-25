#include "NavGraphComponent.h"
#include "Navigation/AgentSpatialInfo.h"


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{

	NavGraphComponent::NavGraphComponent(std::shared_ptr<Simulator> simulator, std::shared_ptr<NavSystem> navSystem)
		: _simulator(simulator), _navSystem(navSystem), _navGraph(navSystem->GetNavGraph()), _pathPlanner(_navGraph)
	{ }

	void NavGraphComponent::AddAgent(size_t id)
	{
		size_t curNodeId = getNodeId(id);
		NavGraphPathPlanner pathPlanner(_navGraph);

		Vector2 curPos = _navSystem->GetSpatialInfo(id).GetPos();
		AgentStruct agtStruct;
		agtStruct.id = id;
		agtStruct.route = pathPlanner.GetRoute(curPos, curPos);
		agtStruct.shiftedRoute = agtStruct.route;
		agtStruct.pointsComplete = 0;
		_agents.push_back(agtStruct);
	}

	void NavGraphComponent::Replan(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentStruct.id);

		agentStruct.route = _pathPlanner.GetRoute(agentInfo.GetPos(), agentGoal.getCentroid());
		agentStruct.shiftedRoute = agentStruct.route;

		agentStruct.pointsComplete = 0;
		agentStruct.goalPoint = agentGoal.getCentroid();
	}

	bool NavGraphComponent::DeleteAgent(size_t id)
	{
		for (int i = 0; i < _agents.size(); i++) {
			if (_agents[i].id == id) {
				_agents.erase(_agents.begin() + i);
				return true;
			}
		}
		return false;
	}

	void NavGraphComponent::Update(float timeStep)
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

			if (IsReplanNeeded(info, agtStruct))
			{
				Replan(info, agtStruct);
			}

			UpdateLocation(info, agtStruct, timeStep);
			SetPrefVelocity(info, agtStruct, timeStep);
		}
	}

	bool NavGraphComponent::IsReplanNeeded(AgentSpatialInfo& agentInfo, AgentStruct& agentStruct)
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentInfo.id);

		return Vector2::DistanceSquared(agentStruct.goalPoint, agentGoal.getCentroid()) > 1e-6f;
	}

	void NavGraphComponent::SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep)
	{
		Vector2 currentGoalShifted = agentStruct.shiftedRoute.points[agentStruct.pointsComplete];
		Vector2 currentGoal = agentStruct.route.points[agentStruct.pointsComplete];
		Vector2 previousGoal = agentStruct.pointsComplete > 0 ? agentStruct.route.points[agentStruct.pointsComplete - 1] : agentStruct.route.points[agentStruct.pointsComplete];
		Vector2 futureGoal = agentStruct.pointsComplete < agentStruct.route.points.size()-1 ? agentStruct.route.points[agentStruct.pointsComplete + 1] : agentStruct.route.points[agentStruct.pointsComplete];

		//Vector2 shift = { -1 * agentInfo.GetOrient().y, agentInfo.GetOrient().x };
		//shift.Normalize();
		//shift *= agentInfo.radius * 4;

		//float dist = Vector2::Distance(currentGoal + shift, agentInfo.GetPos());

		int initialLinesShift = 4;
		int linesShift = 4;
		int calculatedShiftShift;
		bool lineShifted = false;
		if (agentStruct.needLineShift)
		{
			auto inEdges	= _navGraph->GetInEdges(_navGraph->GetClosestNodeIdByPosition(currentGoal, _navGraph->GetAllNodes()));
			auto outEdges	= _navGraph->GetOutEdges(_navGraph->GetClosestNodeIdByPosition(currentGoal, _navGraph->GetAllNodes()));

			auto previousNodeId = _navGraph->GetClosestNodeIdByPosition(previousGoal, _navGraph->GetAllNodes());
			auto currentNodeId = _navGraph->GetClosestNodeIdByPosition(currentGoal, _navGraph->GetAllNodes());

			for (auto& outEdge : outEdges)
			{
				if (outEdge.nodeTo == previousNodeId)
				{
					//agentStruct.currentEdgeIsOneSided = false;
					agentInfo.isCurrentLineOneWay = false;
					break;
				}
			}

			NavGraphEdge currentEdge;
			bool edgeFound = false;
			for (auto& inEdge : inEdges)
			{
				if (inEdge.nodeFrom == previousNodeId)
				{
					currentEdge = inEdge;
					edgeFound = true;
					break;
				}
			}
			if (edgeFound)
			{
				if (currentEdge.weight < agentInfo.currentLine)
				{
					auto halfWeight = (int)currentEdge.weight / 2;
					if (agentInfo.isCurrentLineOneWay)
					{
						agentInfo.currentLine = (currentEdge.weight == 1) ? 0 : (rand() % (int)(currentEdge.weight));
						initialLinesShift = ((int)currentEdge.weight % 2 == 0 ? -4 : 0) - halfWeight * linesShift;
					}
					else
					{
						agentInfo.currentLine = (halfWeight <= 1) ? 0 : (rand() % (int)(halfWeight));
						initialLinesShift = 4;
					}
				}
				agentStruct.needLineShift = false;
				lineShifted = true;
			}
		}

		float turnDirection = ((currentGoal.x - previousGoal.x) * (futureGoal.y - previousGoal.y) - (currentGoal.y - previousGoal.y) * (futureGoal.x - previousGoal.x)) > 0 ? -1 : 1;

		calculatedShiftShift = initialLinesShift + agentInfo.currentLine * linesShift;

		//Vector2 shift = { -1 * agentInfo.GetOrient().y, agentInfo.GetOrient().x };
		//shift.Normalize();
		//shift *= agentInfo.radius * calculatedShiftShift;

		Vector2 turnShift = currentGoal - previousGoal;
		turnShift.Normalize();
		Vector2 futureShift = futureGoal - currentGoal;
		futureShift.Normalize();
		float dot = turnShift.Dot(futureShift);
		float shiftCorrection = dot > 0 ? 1 - dot : 1;
		Vector2 shift = { -1 * turnShift.y, turnShift.x };
		turnShift *= agentInfo.radius * calculatedShiftShift * turnDirection * shiftCorrection;
		shift *= agentInfo.radius * calculatedShiftShift;

		if (lineShifted)
		{
			agentStruct.shiftedRoute.points[agentStruct.pointsComplete] = currentGoal + shift + turnShift;
		}

		float dist = Vector2::Distance(currentGoalShifted/* + shift + turnShift*/, agentInfo.GetPos());

		if(dist < agentInfo.prefSpeed * timeStep)
		{
			agentInfo.prefVelocity.setSpeed(dist);
		} 
		else
		{
			agentInfo.prefVelocity.setSpeed(agentInfo.prefSpeed);
		}

		if (abs(dist) > 1e-6)
		{
			//agentInfo.prefVelocity.setSingle((currentGoal - agentInfo.GetPos()) / dist);
			agentInfo.prefVelocity.setSingle((currentGoalShifted/* + shift + turnShift*/ - agentInfo.GetPos()) / dist);
		} else
		{
			agentInfo.prefVelocity.setSpeed(0);
		}
		//agentInfo.prefVelocity.setTarget(currentGoal);

		TrafficLightsBunch* curLights = _navSystem->GetTrafficLights(_navGraph->GetClosestNodeIdByPosition(currentGoal, _navGraph->GetAllNodes()));
		if (curLights)
		{
			if ((curLights->GetProperLight(agentInfo.GetOrient())->GetCurLight() == TrafficLight::Lights::red ||
				curLights->GetProperLight(agentInfo.GetOrient())->GetCurLight() == TrafficLight::Lights::yellow) &&
				dist < agentInfo.radius * 15 && dist > agentInfo.radius * 12)
			{
				agentInfo.prefVelocity.setSpeed(1e-6);
			}
		}

		agentInfo.prefVelocity.setTarget(currentGoalShifted/* + shift + turnShift*/);
	}

	DirectX::SimpleMath::Vector2 NavGraphComponent::GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p)
	{
		return p;
	}

	void NavGraphComponent::UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, float deltaTime) const
	{
		Vector2 oldPos = agentInfo.GetPos() - agentInfo.GetVel() * deltaTime;

		auto destination = agentStruct.shiftedRoute.points.size() > agentStruct.pointsComplete? agentStruct.shiftedRoute.points[agentStruct.pointsComplete] : agentStruct.route.points[agentStruct.pointsComplete];

		float point_dist = Math::distanceToSegment(oldPos, agentInfo.GetPos(), destination);
		if (abs(point_dist) < acceptanceRadius && agentStruct.pointsComplete < agentStruct.route.points.size() - 1) {
			agentStruct.pointsComplete++;
			agentStruct.needLineShift = true;
			//agentStruct.currentEdgeIsOneSided = true;
			agentInfo.isCurrentLineOneWay = true;
		}
	}

	std::shared_ptr<NavGraph> NavGraphComponent::GetNavGraph() const
	{
		return _navGraph;
	}

	size_t NavGraphComponent::getNodeId(size_t agentId) const
	{
		AgentSpatialInfo & agentInfo = _simulator->GetSpatialInfo(agentId);
		return _navGraph->GetClosestNodeIdByPosition(agentInfo.GetPos(), _navGraph->GetAllNodes());
	}

	unsigned int NavGraphComponent::getGoalNodeId(size_t agentId) const
	{
		auto & agentGoal = _simulator->GetAgentGoal(agentId);
		return _navGraph->GetClosestNodeIdByPosition(agentGoal.getCentroid(), _navGraph->GetAllNodes());
	}
}