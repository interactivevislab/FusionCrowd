#include "NavSystem.h"


#include "Math/Util.h"
#include "TacticComponent/NavMeshComponent.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/Modification/ModificationProcessor.h"
#include "Navigation/NavMesh//Modification/PolygonPreprocessor.h"
#include "Navigation/NavMesh//Modification/EdgeObstacleReplaner.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/FastFixedRadiusNearestNeighbors/NeighborsSeeker.h"

#include <limits>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <math.h>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class NavSystem::NavSystemImpl
	{
	public:
		NavSystemImpl() { }

		~NavSystemImpl() { }

		void SetNavMesh(std::shared_ptr<NavMeshLocalizer> localizer)
		{
			_localizer = localizer;
			_navMeshQuery = std::make_unique<NavMeshSpatialQuery>(localizer);
			_navMesh = localizer->getNavMesh();
		}

		void SetNavGraph(std::unique_ptr<NavGraph> navGraph)
		{
			_navGraph = std::move(navGraph);
		}

		NavGraph* GetNavGraph()
		{
			return _navGraph.get();
		}
		//TEST METHOD, MUST BE DELETED
		int CountNeighbors(size_t agentId) const
		{
			auto neighbors = _agentsNeighbours.find(agentId);
			if (neighbors != _agentsNeighbours.end()) {
				return neighbors->second.size();
			}
			else
			{
				return 0;
			}
		}

		void SetAgentsSensitivityRadius(float radius)
		{
			_agentsSensitivityRadius = radius;
		}

		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position)
		{
			AgentSpatialInfo info;
			info.id = agentId;
			info.pos = position;

			AddAgent(info);
		}

		void AddAgent(AgentSpatialInfo spatialInfo)
		{
			if(spatialInfo.collisionsLevel == AgentSpatialInfo::AGENT)
				_numAgents++;
			else
				_numGroups++;

			_agentsInfo[spatialInfo.id] = spatialInfo;
			_agentsNeighbours[spatialInfo.id] = std::vector<NeighborInfo>();
		}

		void RemoveAgent(unsigned int id) {
			if (_agentsInfo[id].collisionsLevel == AgentSpatialInfo::AGENT)
				_numAgents--;
			else
				_numGroups--;
			_agentsInfo.erase(id);
		}

		AgentSpatialInfo & GetSpatialInfo(size_t agentId)
		{
			return _agentsInfo.at(agentId);
		}

		std::map<size_t, AgentSpatialInfo> GetAgentsSpatialInfos() {
			return _agentsInfo;
		}

		std::vector<NeighborInfo> GetNeighbours(size_t agentId) const
		{
			auto cache = _agentsNeighbours.find(agentId);

			if(cache == _agentsNeighbours.end())
				return std::vector<NeighborInfo>();

			return cache->second;
		}

		std::vector<Obstacle> GetClosestObstacles(size_t agentId)
		{
			std::vector<Obstacle> result;
			AgentSpatialInfo & agent = _agentsInfo.at(agentId);

			if ((agent.useNavMeshObstacles) && (_navMesh != NULL))
			{
				size_t nodeId = _localizer->getNodeId(agent.pos);
				if (nodeId == NavMeshLocation::NO_NODE)
					return result;

				for (size_t obstId : _navMeshQuery->ObstacleQuery(agent.pos))
				{
					result.push_back(_navMesh->GetObstacle(obstId));
				}
			}
			return result;
		}

		void Update(float timeStep)
		{
			for (auto & info : _agentsInfo)
			{
				AgentSpatialInfo & currentInfo = info.second;

				UpdatePos(currentInfo, timeStep);
				UpdateOrient(currentInfo, timeStep);
			}

			UpdateNeighbours();
		}

		void UpdatePos(AgentSpatialInfo & agent, float timeStep)
		{
			float delV = (agent.vel - agent.velNew).Length();
			if (isnan(delV)) {
				agent.velNew = agent.vel;
				delV = 0;
			}

			if (agent.inertiaEnabled && delV > agent.maxAccel * timeStep) {
				float w = agent.maxAccel * timeStep / delV;
				agent.vel = (1.f - w) * agent.vel + w * agent.velNew;
			}
			else {
				agent.vel = agent.velNew;
			}

			agent.pos += agent.vel * timeStep;
		}

		void UpdateOrient(AgentSpatialInfo & agent, float timeStep)
		{
			if(agent.vel.Length() < MathUtil::EPS)
			{
				agent.orient = Vector2(0, 1);
				return;
			}

			agent.orient = agent.vel;
			agent.orient.Normalize();
			return;

			if(!agent.inertiaEnabled)
			{
				agent.orient = agent.vel;
				agent.orient.Normalize();
				return;
			}

			float speed = agent.vel.Length();
			if (abs(speed) <= MathUtil::EPS)
			{
				speed = speed < 0 ? -MathUtil::EPS : MathUtil::EPS;
			}

			const float speedThresh = agent.prefSpeed / 3.f;
			Vector2 newOrient(agent.orient); // by default new is old
			Vector2 moveDir = agent.vel / speed;
			if (speed >= speedThresh)
			{
				newOrient = moveDir;
			}
			else
			{
				float frac = sqrtf(speed / speedThresh);
				Vector2 prefDir = agent.prefVelocity.getPreferred();
				// prefDir *can* be zero if we've arrived at goal.  Only use it if it's non-zero.
				if (prefDir.LengthSquared() > 0.000001f)
				{
					newOrient = frac * moveDir + (1.f - frac) * prefDir;
					newOrient.Normalize();
				}
			}

			// Now limit angular velocity.
			const float MAX_ANGLE_CHANGE = timeStep * agent.maxAngVel;
			float maxCt = cos(MAX_ANGLE_CHANGE);
			float ct = newOrient.Dot(agent.orient);
			if (ct < maxCt)
			{
				// changing direction at a rate greater than _maxAngVel
				float maxSt = sin(MAX_ANGLE_CHANGE);
				if (MathUtil::det(agent.orient, newOrient) > 0.f)
				{
					// rotate _orient left
					agent.orient = Vector2(
						maxCt * agent.orient.x - maxSt * agent.orient.y,
						maxSt * agent.orient.x + maxCt * agent.orient.y
					);
				}
				else
				{
					// rotate _orient right
					agent.orient = Vector2(
						maxCt * agent.orient.x + maxSt * agent.orient.y,
						-maxSt * agent.orient.x + maxCt * agent.orient.y
					);
				}
			}
			else
			{
				agent.orient = newOrient;
			}
		}

		void UpdateNeighbours()
		{
			std::vector<NeighborsSeeker::SearchRequest> agentRequests;
			std::vector<NeighborsSeeker::SearchRequest> groupRequests;

			for (auto & info : _agentsInfo)
			{
				if(info.second.collisionsLevel == AgentSpatialInfo::AGENT)
				{
					agentRequests.push_back(info.second);
				} else
				{
					groupRequests.push_back(info.second);
				}
			}

			for(const auto& p : _neighborsSeeker.FindNeighborsCpu(agentRequests))
			{
				_agentsNeighbours[p.first] = p.second;
			}

			for(const auto& p : _neighborsSeeker.FindNeighborsCpu(groupRequests))
			{
				_agentsNeighbours[p.first] = p.second;
			}
		}

		void Init() {
			UpdateNeighbours();
		}

		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
			auto query = _navMeshQuery.get();
			auto processor = ModificationProcessor(*_navMesh, _localizer, query);
			PolygonPreprocessor pp(polygon);
			auto res =  pp.performAll(processor);
			//EdgeObstacleReplaner(*_navMesh, _localizer).Replan();
			return res;
		}

		INavMeshPublic* GetPublicNavMesh() const
		{
			return _navMesh.get();
		}

	private:
		std::unordered_map<size_t, std::vector<NeighborInfo>> _agentsNeighbours;
		std::unique_ptr<NavMeshSpatialQuery> _navMeshQuery;
		std::shared_ptr<NavMesh> _navMesh;
		std::shared_ptr<NavGraph> _navGraph;
		std::shared_ptr<NavMeshLocalizer> _localizer;

		NeighborsSeeker _neighborsSeeker;
		std::map<size_t, AgentSpatialInfo> _agentsInfo;
		float _agentsSensitivityRadius = 6;
		float _groupSensitivityRadius = 100;

		size_t _numAgents = 0;
		size_t _numGroups = 0;
	};

	NavSystem::NavSystem()
		: pimpl(spimpl::make_unique_impl<NavSystemImpl>())
	{
	}

	void NavSystem::AddAgent(size_t agentId, Vector2 position)
	{
		pimpl->AddAgent(agentId, position);
	}

	void NavSystem::AddAgent(AgentSpatialInfo spatialInfo)
	{
		pimpl->AddAgent(spatialInfo);
	}

	void NavSystem::RemoveAgent(unsigned int id)
	{
		pimpl->RemoveAgent(id);
	}

	AgentSpatialInfo & NavSystem::GetSpatialInfo(size_t agentId)
	{
		return pimpl->GetSpatialInfo(agentId);
	}

	std::map<size_t, AgentSpatialInfo> NavSystem::GetAgentsSpatialInfos() {
		return pimpl->GetAgentsSpatialInfos();
	}

	std::vector<NeighborInfo> NavSystem::GetNeighbours(size_t agentId) const
	{
		return pimpl->GetNeighbours(agentId);
	}

	//TEST METHOD, MUST BE DELETED
	int NavSystem::CountNeighbors(size_t agentId) const {
		return pimpl->CountNeighbors(agentId);
	}

	std::vector<Obstacle> NavSystem::GetClosestObstacles(size_t agentId)
	{
		return pimpl->GetClosestObstacles(agentId);
	}

	void NavSystem::Update(float timeStep)
	{
		pimpl->Update(timeStep);
	}

	void NavSystem::SetAgentsSensitivityRadius(float radius) {
		pimpl->SetAgentsSensitivityRadius(radius);
	}

	void NavSystem::Init() {
		pimpl->Init();
	}


	INavMeshPublic* NavSystem::GetPublicNavMesh() const
	{
		return pimpl->GetPublicNavMesh();
	}

	float NavSystem::CutPolygonFromMesh(FCArray<NavMeshVetrex>& polygon)
	{
		return pimpl->CutPolygonFromMesh(polygon);
	}

	void NavSystem::SetNavMesh(std::shared_ptr<NavMeshLocalizer> localizer)
	{
		pimpl->SetNavMesh(localizer);
	}
	void NavSystem::SetNavGraph(std::unique_ptr<NavGraph> navGraph)
	{
		pimpl->SetNavGraph(std::move(navGraph));
	}
	NavGraph* NavSystem::GetNavGraph()
	{
		return pimpl->GetNavGraph();
	}
}
