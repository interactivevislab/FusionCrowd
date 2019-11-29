#include "NavSystem.h"


#include "Math/Util.h"
#include "TacticComponent/NavMeshComponent.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshModifyer.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Navigation/FastFixedRadiusNearestNeighbors/NeighborsSeeker.h"

#include <limits>
#include <unordered_map>
#include <map>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class NavSystem::NavSystemImpl
	{
	public:
		NavSystemImpl(std::shared_ptr<NavMeshLocalizer> localizer)
		{
			_localizer = localizer;
			_navMeshQuery = std::make_unique<NavMeshSpatialQuery>(localizer);
			_navMesh = localizer->getNavMesh();
		}

		~NavSystemImpl() { }

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
			_agentsInfo[agentId] = info;
		}

		void AddAgent(AgentSpatialInfo spatialInfo)
		{
			_agentsInfo[spatialInfo.id] = spatialInfo;
		}

		AgentSpatialInfo & GetSpatialInfo(size_t agentId)
		{
			return _agentsInfo.at(agentId);
		}

		std::map<size_t, AgentSpatialInfo> GetAgentsSpatialInfos() {
			return _agentsInfo;
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
			AgentSpatialInfo & agent = _agentsInfo.at(agentId);

			std::vector<Obstacle> result;
			for(size_t obstId : _navMeshQuery->ObstacleQuery(agent.pos))
			{
				result.push_back(_navMesh->GetObstacle(obstId));
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
			int numAgents = _agentsInfo.size();

			if(numAgents < 2)
				return;

			std::vector<AgentSpatialInfo> agentsInfos;
			agentsInfos.reserve(numAgents);
			for (auto info : _agentsInfo) {
				agentsInfos.push_back(info.second);
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

			auto allNeighbors = _neighborsSeeker.FindNeighbors(agentsPositions, numAgents, maxX - minX, maxY - minY, _agentsSensitivityRadius, true);
			//NeighborsSeeker::PointNeighbors *allNeighbors = new NeighborsSeeker::PointNeighbors[numAgents];
			//for (int i = 0; i < numAgents; i++) {
			//	allNeighbors[i] = { i, 0 };
			//}

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

		//nav mesh draw export
		size_t GetVertexCount() {
			return _navMesh->GetVertexCount();
		}

		bool GetVertices(FCArray<NavMeshVetrex> & output) {
			return _navMesh->GetVertices(output);
		}

		size_t GetNodesCount() {
			return _navMesh->GetNodesCount();
		}

		size_t GetNodeVertexCount(size_t node_id) {
			return _navMesh->GetNodeVertexCount(node_id);
		}

		bool GetNodeVertexInfo(FCArray<int> & output, size_t node_id) {
			return _navMesh->GetNodeVertexInfo(output, node_id);
		}

		size_t GetEdgesCount() {
			return _navMesh->GetEdgesCount();
		}

		bool GetEdges(FCArray<EdgeInfo> & output) {
			return _navMesh->GetEdges(output);
		}

		size_t GetObstaclesCount() {
			return _navMesh->GetObstaclesCount();
		}

		bool GetObstacles(FCArray<EdgeInfo> & output) {
			return _navMesh->GetObstacles(output);
		}

		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
			return NavMeshModifyer(*_navMesh, _localizer).CutPolygonFromMesh(polygon);
		}

	private:
		std::unordered_map<size_t, std::vector<AgentSpatialInfo>> _agentsNeighbours;
		std::unique_ptr<NavMeshSpatialQuery> _navMeshQuery;
		std::shared_ptr<NavMesh> _navMesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;

		NeighborsSeeker _neighborsSeeker;
		std::map<size_t, AgentSpatialInfo> _agentsInfo;
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

	std::map<size_t, AgentSpatialInfo> NavSystem::GetAgentsSpatialInfos() {
		return pimpl->GetAgentsSpatialInfos();
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

	void NavSystem::Init() {
		pimpl->Init();
	}

	void NavSystem::SetGridCoeff(float coeff) {
		pimpl->SetGridCoeff(coeff);
	}


	//nav mesh draw export
	size_t NavSystem::GetVertexCount() {
		return pimpl->GetVertexCount();
	}

	bool NavSystem::GetVertices(FCArray<NavMeshVetrex> & output) {
		return pimpl->GetVertices(output);
	}


	size_t NavSystem::GetNodesCount() {
		return pimpl->GetNodesCount();
	}

	size_t NavSystem::GetNodeVertexCount(size_t node_id) {
		return pimpl->GetNodeVertexCount(node_id);
	}

	bool NavSystem::GetNodeVertexInfo(FCArray<int> & output, size_t node_id) {
		return pimpl->GetNodeVertexInfo(output, node_id);
	}

	size_t NavSystem::GetEdgesCount() {
		return pimpl->GetEdgesCount();
	}

	bool NavSystem::GetEdges(FCArray<EdgeInfo> & output) {
		return pimpl->GetEdges(output);
	}

	size_t NavSystem::GetObstaclesCount() {
		return pimpl->GetObstaclesCount();
	}

	bool NavSystem::GetObstacles(FCArray<EdgeInfo> & output) {
		return pimpl->GetObstacles(output);
	}

	float NavSystem::CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
		return pimpl->CutPolygonFromMesh(polygon);
	}

}
