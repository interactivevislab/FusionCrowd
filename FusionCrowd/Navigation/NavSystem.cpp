#include "NavSystem.h"


#include "Math/Util.h"
#include "Math/consts.h"
#include "TacticComponent/NavMesh/NavMeshComponent.h"

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

		void SetAgentsSensitivityRadius(float radius)
		{
			_agentsSensitivityRadius = radius;
		}

		void AddAgent(AgentSpatialInfo spatialInfo)
		{
			if(_agentsInfo.find(spatialInfo.id) != _agentsInfo.end())
				return;

			if(spatialInfo.type == AgentSpatialInfo::AGENT)
				_numAgents++;
			else
				_numGroups++;

			_agentsInfo[spatialInfo.id] = std::move(spatialInfo);
			_agentsNeighbours[spatialInfo.id] = std::vector<NeighborInfo>();
		}

		void RemoveAgent(size_t id)
		{
			if(_agentsInfo.erase(id))
			{
				if (_agentsInfo[id].type == AgentSpatialInfo::AGENT)
					_numAgents--;
				else
					_numGroups--;
			}
		}

		AgentSpatialInfo & GetSpatialInfo(size_t agentId)
		{
			return _agentsInfo.at(agentId);
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
				size_t nodeId = _localizer->getNodeId(agent.GetPos());
				if (nodeId == NavMeshLocation::NO_NODE)
					return result;

				for (size_t obstId : _navMeshQuery->ObstacleQuery(agent.GetPos()))
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

				Vector2 newPos, newVel, newOrient;
				UpdatePos(currentInfo, timeStep, newPos, newVel);
				UpdateOrient(currentInfo, timeStep, newOrient);

				currentInfo.Update(newPos, newVel, newOrient);
			}

			UpdateNeighbours();
		}

		void UpdatePos(AgentSpatialInfo & agent, float timeStep, Vector2 & updatedPos, Vector2 & updatedVel)
		{
			const float delV = (agent.GetVel() - agent.velNew).Length();

			if (isnan(delV))
			{
				updatedVel = agent.GetVel();
				updatedPos = agent.GetPos();

				return;
			}

			if (agent.inertiaEnabled && delV > agent.maxAccel * timeStep)
			{
				const float w = agent.maxAccel * timeStep / delV;
				updatedVel = (1.f - w) * agent.GetVel() + w * agent.velNew;
			}
			else
			{
				updatedVel = agent.velNew;
			}

			updatedPos = agent.GetPos() + agent.GetVel() * timeStep;
			updatedPos = _localizer->GetClosestAvailablePoint(updatedPos);
		}

		void UpdateOrient(const AgentSpatialInfo & agent, float timeStep, Vector2 & newOrient)
		{
			float speed = agent.GetVel().Length();
			if(speed < Math::EPS)
			{
				newOrient = Vector2(0, 1);
				return;
			}

			if(!agent.inertiaEnabled)
			{
				agent.GetVel().Normalize(newOrient);
				return;
			}

			if (abs(speed) <= Math::EPS)
			{
				speed = speed < 0 ? -Math::EPS : Math::EPS;
			}

			const float speedThresh = agent.prefSpeed / 3.f;

			Vector2 moveDir = agent.GetVel() / speed;
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
			float ct = newOrient.Dot(agent.GetOrient());
			if (ct < maxCt)
			{
				// changing direction at a rate greater than _maxAngVel
				float maxSt = sin(MAX_ANGLE_CHANGE);
				if (Math::det(agent.GetOrient(), newOrient) > 0.f)
				{
					// rotate _orient left
					newOrient = Vector2(
						maxCt * agent.GetOrient().x - maxSt * agent.GetOrient().y,
						maxSt * agent.GetOrient().x + maxCt * agent.GetOrient().y
					);
				}
				else
				{
					// rotate _orient right
					newOrient = Vector2(
						maxCt * agent.GetOrient().x + maxSt * agent.GetOrient().y,
						-maxSt * agent.GetOrient().x + maxCt * agent.GetOrient().y
					);
				}
			}
		}

		void UpdateNeighbours()
		{
			std::vector<NeighborsSeeker::SearchRequest> agentRequests;

			for (auto & info : _agentsInfo)
			{
				agentRequests.push_back(info.second);
			}

			for(const auto& p : _neighborsSeeker.FindNeighborsCpu(agentRequests))
			{
				_agentsNeighbours[p.agentId] = p.neighbors;
				_agentsInfo[p.agentId].setOverlaping(p.isOverlapped);
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

	void NavSystem::AddAgent(AgentSpatialInfo spatialInfo)
	{
		pimpl->AddAgent(std::move(spatialInfo));
	}

	void NavSystem::RemoveAgent(size_t id)
	{
		pimpl->RemoveAgent(id);
	}

	AgentSpatialInfo & NavSystem::GetSpatialInfo(size_t agentId)
	{
		return pimpl->GetSpatialInfo(agentId);
	}

	std::vector<NeighborInfo> NavSystem::GetNeighbours(size_t agentId) const
	{
		return pimpl->GetNeighbours(agentId);
	}

	std::vector<Obstacle> NavSystem::GetClosestObstacles(size_t agentId)
	{
		return pimpl->GetClosestObstacles(agentId);
	}

	void NavSystem::Update(float timeStep)
	{
		pimpl->Update(timeStep);
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
