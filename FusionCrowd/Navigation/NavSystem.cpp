#include "NavSystem.h"


#include "Math/Util.h"
#include "TacticComponent/NavMeshComponent.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/FastFixedRadiusNearestNeighbors/NeighborsSeeker.h"
#include "Navigation/OnlineRecording/OnlineRecording.h"

#include <limits>
#include <unordered_map>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class NavSystem::NavSystemImpl
	{
	public:
		NavSystemImpl(std::shared_ptr<NavMeshLocalizer> localizer)
		{
			_recording = OnlineRecording();
			_navMeshQuery = std::make_unique<NavMeshSpatialQuery>(localizer);
			_navMesh = localizer->getNavMesh();
		}

		~NavSystemImpl() { }

		IRecording & GetRecording()
		{
			return _recording;
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

			_recording.AddAgent(info);
		}

		void AddAgent(AgentSpatialInfo spatialInfo)
		{
			_recording.AddAgent(spatialInfo);
		}

		AgentSpatialInfo & GetSpatialInfo(size_t agentId)
		{
			return _recording.GetCurrentSpatialInfo(agentId);
		}

		const std::vector<AgentSpatialInfo> & GetNeighbours(size_t agentId) const
		{
			auto result = _agentsNeighbours.find(agentId);

			if(result == _agentsNeighbours.end())
				return std::vector<AgentSpatialInfo>();

			return result->second;
		}

		std::vector<Obstacle> GetClosestObstacles(size_t agentId)
		{
			AgentSpatialInfo & agent = _recording.GetCurrentSpatialInfo(agentId);

			std::vector<Obstacle> result;
			for(size_t obstId : _navMeshQuery->ObstacleQuery(agent.pos))
			{
				result.push_back(_navMesh->GetObstacle(obstId));
			}

			return result;
		}

		void Update(float timeStep)
		{
			FCArray<size_t> ids(_recording.GetAgentCount());
			_recording.GetAgentIds(ids);
			for (size_t id : ids)
			{
				AgentSpatialInfo & info = _recording.GetCurrentSpatialInfo(id);

				UpdatePos(info, timeStep);
				UpdateOrient(info, timeStep);
			}

			_recording.Update(timeStep);

			UpdateNeighbours();
		}

		void UpdatePos(AgentSpatialInfo & agent, float timeStep)
		{
			float delV = (agent.vel - agent.velNew).Length();

			if (delV > agent.maxAccel * timeStep) {
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
			int numAgents = _recording.GetAgentCount();

			if(numAgents < 2)
				return;

			std::vector<AgentSpatialInfo> agentsInfos;
			agentsInfos.reserve(numAgents);
			FCArray<size_t> ids(_recording.GetAgentCount());
			_recording.GetAgentIds(ids);
			for (size_t id : ids) {
				agentsInfos.push_back(_recording.GetCurrentSpatialInfo(id));
			}

			float minX = std::numeric_limits<float>::max();
			float minY = std::numeric_limits<float>::max();
			float maxX = std::numeric_limits<float>::min();
			float maxY = std::numeric_limits<float>::min();
			for (auto & info : agentsInfos) {
				if (info.pos.x < minX) minX = info.pos.x;
				if (info.pos.x > maxX) maxX = info.pos.x;
				if (info.pos.y < minY) minY = info.pos.y;
				if (info.pos.y > maxY) maxY = info.pos.y;
			}

			Point *agentsPositions = new Point[numAgents];
			int i = 0;
			for (auto & info : agentsInfos) {
				agentsPositions[i].x = info.pos.x - minX;
				agentsPositions[i].y = info.pos.y - minY;
				i++;
			}

			_neighborsSeeker.Init(agentsPositions, numAgents, maxX - minX, maxY - minY, _agentsSensitivityRadius);

			auto allNeighbors = _neighborsSeeker.FindNeighbors(true);

			_agentsNeighbours.reserve(numAgents);
			i = 0;

			for (auto & info : agentsInfos) {

				auto dataPair = _agentsNeighbours.find(info.id);
				if (dataPair == _agentsNeighbours.end()) {
					dataPair = _agentsNeighbours.insert({ info.id, std::vector<AgentSpatialInfo>() }).first;
				}
				else
				{
					dataPair->second.clear();
				}

				auto neighbors = allNeighbors[i];

				auto &neighborsInfos = dataPair->second;
				neighborsInfos.reserve(neighbors.neighborsCount);

				for (int j = 0; j < neighbors.neighborsCount; j++) {
					neighborsInfos.push_back(agentsInfos[neighbors.neighborsID[j]]);
				}

				i++;
			}

			delete[] agentsPositions;
		}

		void Init() {
			UpdateNeighbours();
		}

		void SetGridCoeff(float coeff) {
			_neighborsSeeker.gridCellCoeff = coeff;
		}

	private:
		std::unordered_map<size_t, std::vector<AgentSpatialInfo>> _agentsNeighbours;
		std::unique_ptr<NavMeshSpatialQuery> _navMeshQuery;
		std::shared_ptr<NavMesh> _navMesh;

		NeighborsSeeker _neighborsSeeker;
		OnlineRecording _recording;
		float _agentsSensitivityRadius = 1;
	};

	NavSystem::NavSystem(std::shared_ptr<NavMeshLocalizer> localizer)
		: pimpl(spimpl::make_unique_impl<NavSystemImpl>(localizer))
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

	AgentSpatialInfo & NavSystem::GetSpatialInfo(size_t agentId)
	{
		return pimpl->GetSpatialInfo(agentId);
	}

	const std::vector<AgentSpatialInfo> & NavSystem::GetNeighbours(size_t agentId) const
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

	IRecording & NavSystem::GetRecording()
	{
		return pimpl->GetRecording();
	}

	void NavSystem::Init() {
		pimpl->Init();
	}

	void NavSystem::SetGridCoeff(float coeff) {
		pimpl->SetGridCoeff(coeff);
	}
}
