#include "Simulator.h"

#include "Export/ComponentId.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMesh/NavMeshComponent.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Navigation/OnlineRecording/OnlineRecording.h"
#include "Group/GridGroup.h"
#include "Group/GuidedGroup.h"
#include "Math/Shapes/ConeShape.h"
#include "Math/Shapes/DiskShape.h"
#include "Math/consts.h"

#include <random>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
#pragma region implementation
	class Simulator::SimulatorImpl
	{
	public:
		SimulatorImpl()
		{
			_recording = OnlineRecording();
		}

		~SimulatorImpl() = default;

		bool DoStep(float timeStep)
		{
			_currentTime += timeStep;

			for (auto & strategy : _strategyComponents)
			{
				strategy.second->Update(timeStep);
			}

			for (auto & tactic : _tacticComponents)
			{
				tactic.second->Update(timeStep);
			}

			SwitchOpComponents();

			for (auto & oper : _operComponents)
			{
				oper.second->Update(timeStep);
			}

			_navSystem->Update(timeStep);
			if (_isRecording) _recording.MakeRecord(GetAgentsInfo(), timeStep);

			return true;
		}

		size_t GetAgentCount() const { return _agents.size(); }

		AgentSpatialInfo & GetSpatialInfo(size_t agentId) {
			return _navSystem->GetSpatialInfo(agentId);
		}

		IRecording & GetRecording() {
			return _recording;
		}

		void SetIsRecording(bool isRecording) {
			_isRecording = isRecording;
		}

		const Goal & GetAgentGoal(size_t agentId) const {
			return _agents.find(agentId)->second.currentGoal;
		}

		size_t AddAgent(Vector2 pos)
		{
			AgentSpatialInfo info;
			info.SetPos(pos);

			return AddAgent(std::move(info), GetAnyOperational(), GetAnyTactic(), ComponentIds::NO_COMPONENT);
		}

		size_t AddAgent(Vector2 pos, ComponentId opId, ComponentId tacticId, ComponentId strategyId)
		{
			// Use default values
			AgentSpatialInfo info;
			info.SetPos(pos);

			std::uniform_real_distribution<float> dist(0.9f, 1.1f);
			info.prefSpeed *= dist(_rnd_seed);
			info.maxSpeed = info.maxSpeed < info.prefSpeed ? info.prefSpeed : info.maxSpeed;

			return AddAgent(std::move(info), opId, tacticId, strategyId);
		}

		size_t AddAgent(
			AgentSpatialInfo props,
			ComponentId opId,
			ComponentId tacticId,
			ComponentId strategyId
		)
		{
			AgentSpatialInfo info = std::move(props);
			auto agentId = GetNextId();
			info.id = agentId;

			std::uniform_real_distribution<float> dist(0.9f, 1.1f);
			info.prefSpeed *= dist(_rnd_seed);

			info.useNavMeshObstacles = (tacticId == ComponentIds::NAVMESH_ID);

			Vector2 goal_pos = _tacticComponents[tacticId]->GetClosestAvailablePoint(info.GetPos());
			_navSystem->AddAgent(std::move(info));

			Agent a(agentId, _goalFactory.CreatePointGoal(goal_pos));
			_agents.insert({ agentId, std::move(a)});

			auto & agent = _agents.find(agentId)->second;

			auto op = _operComponents.find(opId);
			if(op != _operComponents.end())
			{
				op->second->AddAgent(agentId);
				agent.opComponent = op->second;
			}

			auto tactic = _tacticComponents.find(tacticId);
			if(tactic != _tacticComponents.end())
			{
				tactic->second->AddAgent(agentId);
				agent.tacticComponent = tactic->second;
			}

			auto strat = _strategyComponents.find(strategyId);
			if(strat != _strategyComponents.end())
			{
				strat->second->AddAgent(agentId);
				agent.stratComponent = strat->second;
			}

			return agentId;
		}

		bool UpdateAgentParams(AgentParams params)
		{
			SetOperationComponent(params.id, params.opCompId);
			SetTacticComponent(params.id, params.tacticCompId);
			auto& si = _navSystem->GetSpatialInfo(params.id);
			si.radius = params.radius;
			si.maxSpeed = params.maxSpeed;
			si.maxAccel = params.maxAccel;
			si.prefSpeed = params.prefSpeed;
			si.maxAngVel = params.maxAngVel;
			si.inertiaEnabled = params.inertiaEnabled;
			si.useNavMeshObstacles = params.useNavMeshObstacles;

			return true;
		}

		bool UpdateNeighbourSearchShape(size_t agentId, Cone cone)
		{
			if(_agents.find(agentId) == _agents.end())
				return false;

			auto & agt = _navSystem->GetSpatialInfo(agentId);
			agt.neighbourSearchShape = std::make_unique<Math::ConeShape>(Vector2(), 0, 0);

			return true;
		}

		bool UpdateNeighbourSearchShape(size_t agentId, Disk disk)
		{
			if(_agents.find(agentId) == _agents.end())
				return false;

			auto & agt = _navSystem->GetSpatialInfo(agentId);
			agt.neighbourSearchShape = std::make_unique<Math::DiskShape>(disk);

			return true;
		}

		OperationStatus RemoveGroup(size_t groupId) {

			if (_groups.find(groupId) == _groups.end())
			{
				return OperationStatus::InvalidArgument;
			}

			auto & g = _groups[groupId];

			for (size_t agentId : g->GetAgents())
			{
				auto a = _agents.find(agentId);
				if (a == _agents.end())
				{
					continue;
				}

				a->second.SetGroupId(IGroup::NO_GROUP);
			}

			_groups.erase(groupId);
			return OperationStatus::OK;
		}

		OperationStatus RemoveAgent(size_t agentId) {
			_navSystem->RemoveAgent(agentId);
			auto & agent = _agents.find(agentId)->second;

			for (auto& c : _operComponents) {
				c.second->DeleteAgent(agentId);
			}

			for (auto& c : _tacticComponents) {
				c.second->DeleteAgent(agentId);
			}

			for (auto& c : _strategyComponents)
			{
				c.second->RemoveAgent(agentId);
			}

			_agents.erase(agentId);
			return OperationStatus::OK;
		}

		void SetAgentGoal(size_t agentId, Goal && goal)
		{
			const auto & agentIt = _agents.find(agentId);
			if (agentIt == _agents.end()) return;

			Agent & agent = agentIt->second;
			if (agent.tacticComponent.expired()) return;
			if (auto tactic = agent.tacticComponent.lock())
			{
				auto point = _tacticComponents[tactic->GetId()]->GetClosestAvailablePoint(goal.getCentroid());
				agent.currentGoal = std::move(goal);
			}
		}

		bool SetOperationComponent(size_t agentId, ComponentId newOperationComponent)
		{
			if(_operComponents.find(newOperationComponent) == _operComponents.end())
			{
				return false;
			}

			_switchComponentTasks.insert({ agentId, newOperationComponent});

			Agent & agent = _agents.find(agentId)->second;

			return true;
		}

		bool SetTacticComponent(size_t agentId, ComponentId newTactic)
		{
			if(_tacticComponents.find(newTactic) == _tacticComponents.end())
			{
				return false;
			}

			Agent & agent = _agents.find(agentId)->second;

			if(auto old = agent.tacticComponent.lock())
			{
				old->DeleteAgent(agentId);
			}

			_tacticComponents[newTactic]->AddAgent(agentId);
			agent.tacticComponent = _tacticComponents[newTactic];

			return true;
		}

		bool SetStrategyComponent(size_t agentId, ComponentId newStrategyComponent)
		{
			if(_strategyComponents.find(newStrategyComponent) == _strategyComponents.end())
			{
				return false;
			}

			auto & agent = _agents.find(agentId)->second;
			if(auto oldStrat = agent.stratComponent.lock())
			{
				oldStrat->RemoveAgent(agentId);
			}

			_strategyComponents[newStrategyComponent]->AddAgent(agentId);
			agent.stratComponent = _strategyComponents[newStrategyComponent];

			return true;
		}

		void AddOperComponent(std::shared_ptr<IOperationComponent> operComponent)
		{
			_operComponents[operComponent->GetId()] = operComponent;
		}

		void AddTacticComponent(std::shared_ptr<ITacticComponent> tacticComponent)
		{
			_tacticComponents[tacticComponent->GetId()] = tacticComponent;
		}

		void AddStrategyComponent(std::shared_ptr<IStrategyComponent> strategyComponent)
		{
			_strategyComponents[strategyComponent->GetId()] = strategyComponent;
		}

		float GetElapsedTime()
		{
			return _currentTime;
		}

		void SetNavSystem(std::shared_ptr<NavSystem> navSystem)
		{
			_navSystem = navSystem;
			_navSystem->Init();
		}

		Agent & GetAgent(size_t id)
		{
			return _agents.find(id)->second;
		}

		FCArray<AgentInfo> GetAgentsInfo()
		{
			FCArray<AgentInfo> output(_agents.size());

			GetAgentsInfo(output);

			return output;
		}

		bool GetAgentsInfo(FCArray<AgentInfo> & output)
		{
			if(output.size() < _agents.size())
			{
				return false;
			}

			int i = 0;
			for(auto & p : _agents)
			{
				Agent & agent = p.second;
				AgentSpatialInfo & info = _navSystem->GetSpatialInfo(agent.id);
				auto & g = agent.currentGoal;

				ComponentId op = -1, tactic = -1, strat = -1;
				if(!agent.opComponent.expired())
				{
					op = agent.opComponent.lock()->GetId();
				}

				if(!agent.tacticComponent.expired())
				{
					tactic = agent.tacticComponent.lock()->GetId();
				}

				if(!agent.stratComponent.expired())
				{
					strat = agent.stratComponent.lock()->GetId();
				}

				output[i] = AgentInfo {
					agent.id,
					info.GetPos().x, info.GetPos().y,
					info.GetVel().x, info.GetVel().y,
					info.GetOrient().x, info.GetOrient().y,
					info.radius,
					op, tactic, strat,
					g.getCentroid().x, g.getCentroid().y
				};
				i++;
			}

			return true;
		}

		void SetAgentStrategyParam(size_t agentId, ComponentId strategyId, ModelAgentParams & params)
		{
			_strategyComponents[strategyId]->SetAgentParams(agentId, params);
		}

		IStrategyComponent* GetStrategy(ComponentId strategyId) const
		{
			return _strategyComponents.find(strategyId)->second.get();
		}

		NavSystem* GetNavSystem() const
		{
			return _navSystem.get();
		}

		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
			float res = _navSystem->CutPolygonFromMesh(polygon);
			for (auto & pair : _agents) {
				SetAgentGoal(pair.first, Goal(pair.second.currentGoal));
			}
			return res;
		}

		size_t AddGridGroup(DirectX::SimpleMath::Vector2 origin, size_t agentsInRow, float interAgentDistance)
		{
			size_t grpId = _nextGroupId++;

			AgentSpatialInfo dummyInfo;
			dummyInfo.collisionsLevel = AgentSpatialInfo::GROUP;
			dummyInfo.type = AgentSpatialInfo::GROUP;
			dummyInfo.maxAngVel = 50.5f;
			dummyInfo.inertiaEnabled = false;
			dummyInfo.neighbourSearchShape = std::make_unique<Math::DiskShape>(Vector2(), 50);
			dummyInfo.SetPos(origin);

			size_t dummyId = AddAgent(std::move(dummyInfo), GetAnyOperational(), GetAnyTactic(), ComponentIds::NO_COMPONENT);

			_groups[grpId] = std::make_unique<GridGroup>(grpId, dummyId, agentsInRow, interAgentDistance);

			return grpId;
		}

		size_t AddGuidedGroup(size_t leaderId)
		{
			size_t grpId = _nextGroupId++;

			auto & leaderInfo = _navSystem->GetSpatialInfo(leaderId);

			AgentSpatialInfo dummyInfo;
			dummyInfo.collisionsLevel = AgentSpatialInfo::GROUP;
			dummyInfo.type = AgentSpatialInfo::GROUP;
			dummyInfo.prefSpeed /= 2.0f;
			dummyInfo.maxSpeed  /= 2.0f;
			dummyInfo.maxAngVel = 50.5f;
			dummyInfo.inertiaEnabled = false;
			dummyInfo.SetPos(leaderInfo.GetPos());

			size_t dummyId = AddAgent(std::move(dummyInfo), GetAnyOperational(), GetAnyTactic(), ComponentIds::NO_COMPONENT);

			_groups[grpId] = std::make_unique<GuidedGroup>(grpId, dummyId, leaderInfo);

			AddAgentToGroup(leaderId, grpId);

			return grpId;
		}

		void SetGroupGoal(size_t groupId, DirectX::SimpleMath::Vector2 goalPos)
		{
			if(_groups.find(groupId) == _groups.end())
			{
				return;
			}

			size_t dummyId = _groups[groupId]->GetDummyId();
			SetAgentGoal(dummyId, _goalFactory.CreatePointGoal(goalPos));
		}

		void AddAgentToGroup(size_t agentId, size_t groupId)
		{
			auto g = _groups.find(groupId);
			auto a = _agents.find(agentId);
			if(g == _groups.end() || a == _agents.end())
			{
				// No such agent or no such group exists
				return;
			}

			auto & agtInfo = _navSystem->GetSpatialInfo(agentId);
			agtInfo.collisionsLevel = AgentSpatialInfo::AGENT;

			a->second.SetGroupId(groupId);
			g->second->AddAgent(agentId, agtInfo);

			auto & dummy = _navSystem->GetSpatialInfo(g->second->GetDummyId());
			dummy.radius = g->second->GetRadius();
		}

		void RemoveAgentFromGroup(size_t agentId, size_t groupId)
		{
			auto g = _groups.find(groupId);
			auto a = _agents.find(agentId);
			if(g == _groups.end() || a == _agents.end())
			{
				// No such agent or no such group exists
				return;
			}

			_navSystem->GetSpatialInfo(agentId).collisionsLevel = AgentSpatialInfo::COLLIDE_ALL;

			a->second.SetGroupId(IGroup::NO_GROUP);
			g->second->RemoveAgent(agentId);
		}

		IGroup* GetGroup(size_t groupId)
		{
			return _groups[groupId].get();
		}

		GoalFactory & GetGoalFactory()
		{
			return _goalFactory;
		}

	private:
		size_t GetNextId()
		{
			return _nextAgentId++;
		}

		void SwitchOpComponents()
		{
			for (auto task : _switchComponentTasks) {

				auto agentId = task.first;
				auto newOperationComponent = _operComponents.find(task.second);

				if(newOperationComponent == _operComponents.end())
				{
					// non-existing operation component, possible bug here
					continue;
				}

				if (_agents.find(agentId) == _agents.end()) continue;
				Agent & agent = _agents.find(agentId)->second;
				if(auto old = agent.opComponent.lock())
				{
					if (old != nullptr)
					{
						old->DeleteAgent(agentId);
					}
				}

				newOperationComponent->second->AddAgent(agentId);
				agent.opComponent = newOperationComponent->second;
			}

			_switchComponentTasks.clear();
		}

		ComponentId GetAnyTactic()
		{
			return _tacticComponents.begin()->first;
		}

		ComponentId GetAnyOperational()
		{
			return _operComponents.begin()->first;
		}
	private:
		size_t _nextAgentId = 0;

		// groupId == 0 means that agent has no group
		size_t _nextGroupId = 1;
		std::map<size_t, std::unique_ptr<IGroup>> _groups;

		std::map<size_t, ComponentId> _switchComponentTasks;

		float _currentTime = 0;

		std::shared_ptr<NavSystem> _navSystem;
		OnlineRecording _recording;
		bool _isRecording = false;

		std::map<size_t, FusionCrowd::Agent> _agents;

		std::map<ComponentId, std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::map<ComponentId, std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::map<ComponentId, std::shared_ptr<IOperationComponent>> _operComponents;

		GoalFactory _goalFactory;

		std::random_device _rnd_device;
		std::mt19937 _rnd_seed;
	};

#pragma endregion

#pragma region proxy methods
	Simulator::Simulator() : pimpl(spimpl::make_unique_impl<SimulatorImpl>())
	{
	}

	bool Simulator::DoStep(float timeStep)
	{
		return pimpl->DoStep(timeStep);
	}

	size_t Simulator::GetAgentCount() const
	{
		return pimpl->GetAgentCount();
	}

	AgentSpatialInfo & Simulator::GetSpatialInfo(size_t agentId) {
		return pimpl->GetSpatialInfo(agentId);
	}

	IRecording & Simulator::GetRecording() {
		return pimpl->GetRecording();
	}

	void Simulator::SetIsRecording(bool isRecording) {
		pimpl->SetIsRecording(isRecording);
	}

	const Goal & Simulator::GetAgentGoal(size_t agentId) const {
		return pimpl->GetAgentGoal(agentId);
	}

	bool Simulator::SetOperationComponent(size_t agentId, ComponentId newOperationComponent)
	{
		return pimpl->SetOperationComponent(agentId, newOperationComponent);
	}

	bool Simulator::SetTacticComponent(size_t agentId, ComponentId newTactic)
	{
		return pimpl->SetTacticComponent(agentId, newTactic);
	}

	bool Simulator::SetStrategyComponent(size_t agentId, ComponentId newStrategyComponent)
	{
		return pimpl->SetStrategyComponent(agentId, newStrategyComponent);
	}

	Simulator & Simulator::AddOpModel(std::shared_ptr<IOperationComponent> component)
	{
		pimpl->AddOperComponent(component);
		return *this;
	}

	Simulator & Simulator::AddTactic(std::shared_ptr<ITacticComponent> component)
	{
		pimpl->AddTacticComponent(component);
		return *this;
	}

	Simulator & Simulator::AddStrategy(std::shared_ptr<IStrategyComponent> component)
	{
		pimpl->AddStrategyComponent(component);
		return *this;
	}

	float Simulator::GetElapsedTime()
	{
		return pimpl->GetElapsedTime();
	}

	Simulator & Simulator::UseNavSystem(std::shared_ptr<NavSystem> system)
	{
		pimpl->SetNavSystem(system);
		return *this;
	}

	Agent & Simulator::GetAgent(size_t id)
	{
		return pimpl->GetAgent(id);
	}

	size_t Simulator::AddAgent(
		Vector2 pos,
		ComponentId opId,
		ComponentId tacticId,
		ComponentId strategyId
	) {
		return pimpl->AddAgent(pos, opId, tacticId, strategyId);
	}

	size_t Simulator::AddAgent(DirectX::SimpleMath::Vector2 pos)
	{
		return pimpl->AddAgent(pos);
	}

	bool Simulator::UpdateAgentParams(AgentParams params) {
		return pimpl->UpdateAgentParams(params);
	}

	bool Simulator::UpdateNeighbourSearchShape(size_t agentId, Cone cone)
	{
		return pimpl->UpdateNeighbourSearchShape(agentId, cone);
	}

	bool Simulator::UpdateNeighbourSearchShape(size_t agentId, Disk disk)
	{
		return pimpl->UpdateNeighbourSearchShape(agentId, disk);
	}

	size_t Simulator::AddAgent(AgentSpatialInfo props, ComponentId opId, ComponentId tacticId, ComponentId strategyId)
	{
		return pimpl->AddAgent(std::move(props), opId, tacticId, strategyId);
	}

	OperationStatus Simulator::RemoveAgent(size_t agentId)
	{
		return pimpl->RemoveAgent(agentId);
	}
	OperationStatus Simulator::RemoveGroup(size_t groupId) {
		return pimpl->RemoveGroup(groupId);
	}

	void Simulator::SetAgentGoal(size_t agentId, Goal && goal)
	{
		pimpl->SetAgentGoal(agentId, std::move(goal));
	}

	FCArray<AgentInfo> Simulator::GetAgentsInfo()
	{
		return pimpl->GetAgentsInfo();
	}

	bool Simulator::GetAgentsInfo(FCArray<AgentInfo> & output)
	{
		return pimpl->GetAgentsInfo(output);
	}

	//nav mesh draw export
	NavSystem* Simulator::GetNavSystem() const
	{
		return pimpl->GetNavSystem();
	}

	void Simulator::SetAgentStrategyParam(size_t agentId, ComponentId strategyId, ModelAgentParams & params)
	{
		pimpl->SetAgentStrategyParam(agentId, strategyId, params);
	}

	IStrategyComponent* Simulator::GetStrategy(ComponentId strategyId) const
	{
		return pimpl->GetStrategy(strategyId);
	}

	size_t Simulator::AddGridGroup(DirectX::SimpleMath::Vector2 origin, size_t agentsInRow, float interAgentDistance)
	{
		return pimpl->AddGridGroup(origin, agentsInRow, interAgentDistance);
	}

	size_t Simulator::AddGuidedGroup(size_t leaderId)
	{
		return pimpl->AddGuidedGroup(leaderId);
	}

	void Simulator::SetGroupGoal(size_t groupId, DirectX::SimpleMath::Vector2 goalPos)
	{
		pimpl->SetGroupGoal(groupId, goalPos);
	}

	void Simulator::AddAgentToGroup(size_t agentId, size_t groupId)
	{
		pimpl->AddAgentToGroup(agentId, groupId);
	}

	void Simulator::RemoveAgentFromGroup(size_t agentId, size_t groupId)
	{
		pimpl->RemoveAgentFromGroup(agentId, groupId);
	}

	IGroup* Simulator::GetGroup(size_t groupId)
	{
		return pimpl->GetGroup(groupId);
	}

	GoalFactory & Simulator::GetGoalFactory()
	{
		return pimpl->GetGoalFactory();
	}
#pragma endregion
}
