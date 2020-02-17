#include "Simulator.h"

#include "Export/ComponentId.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMeshComponent.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Navigation/OnlineRecording/OnlineRecording.h"

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

			for (auto tactic : _tacticComponents)
			{
				tactic->Update(timeStep);
			}

			SwitchOpComponents();

			for (auto oper : _operComponents)
			{
				oper->Update(timeStep);
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

		size_t AddAgent(DirectX::SimpleMath::Vector2 pos)
		{
			AgentSpatialInfo info;
			info.id = GetNextId();
			info.pos = pos;
			_navSystem->AddAgent(info);

			Agent a(info.id, _goalFactory.CreatePointGoal(pos));
			_agents.insert({info.id, a});

			return info.id;
		}

		size_t AddAgent(
			float x, float y,
			ComponentId opId,
			ComponentId strategyId,
			ComponentId tacticId
		)
		{
			AgentSpatialInfo info;
			auto agentId = GetNextId();
			info.id = agentId;
			DirectX::SimpleMath::Vector2 goal_pos = _tacticComponents[0]->GetClosestAvailablePoint(DirectX::SimpleMath::Vector2(x, y));
			info.pos = goal_pos;
			_navSystem->AddAgent(info);

			Agent a(agentId, _goalFactory.CreatePointGoal(goal_pos));
			_agents.insert({ agentId, a});

			auto & agent = _agents.find(agentId)->second;

			//SetOperationComponent(info.id, opId);
			for (auto& c : _operComponents) {
				if (c->GetId() == opId) {
					c->AddAgent(agentId);
					agent.opComponent = c;
					break;
				}
			}

			//SetTacticComponent(agentId, ComponentIds::NAVMESH_ID);
			for (auto& c : _tacticComponents) {
				if (c->GetId() == tacticId) {
					c->AddAgent(agentId);
					agent.tacticComponent = c;
					break;
				}
			}

			//SetStrategyComponent(agentId, strategyId);

			if(_strategyComponents.find(strategyId) != _strategyComponents.end())
			{
				auto & c = _strategyComponents[strategyId];
				c->AddAgent(agentId);
				agent.stratComponent = c;
			}


			return agentId;
		}

		void SetAgentGoal(Agent & agent, DirectX::SimpleMath::Vector2 goalPos)
		{
			auto point = _tacticComponents[0]->GetClosestAvailablePoint(goalPos);
			auto goal = _goalFactory.CreateDiscGoal(point, 1.5f);
			agent.currentGoal = goal;
		}

		void SetAgentGoal(size_t agentId, DirectX::SimpleMath::Vector2 goalPos)
		{
			SetAgentGoal(_agents.find(agentId)->second, goalPos);
		}

		bool SetOperationComponent(size_t agentId, ComponentId newOperationComponent)
		{
			for (auto& c : _operComponents){
				if (c->GetId() == newOperationComponent) {
					_switchComponentTasks.insert({ agentId, newOperationComponent });
					return true;
				}
			}
			return false;
		}

		bool SetTacticComponent(size_t agentId, ComponentId newTactic)
		{
			for(auto& c : _tacticComponents)
			{
				if(c->GetId() == newTactic) {
					Agent & agent = _agents.find(agentId)->second;

					if(!agent.tacticComponent.expired())
					{
						agent.tacticComponent.lock()->DeleteAgent(agentId);
					}

					c->AddAgent(agentId);
					agent.tacticComponent = c;

					return true;
				}
			}
			return false;
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
			_operComponents.push_back(operComponent);
		}

		void AddTacticComponent(std::shared_ptr<ITacticComponent> tacticComponent)
		{
			_tacticComponents.push_back(tacticComponent);
		}

		void AddStrategyComponent(std::shared_ptr<IStrategyComponent> strategyComponent)
		{
			_strategyComponents[strategyComponent->GetId()] = strategyComponent;
		}

		void InitSimulator() {
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
					info.pos.x, info.pos.y,
					info.vel.x, info.vel.y,
					info.orient.x, info.orient.y,
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
			for (auto pair : _agents) {
				SetAgentGoal(pair.first, pair.second.currentGoal.getCentroid());
			}
			return res;
		}

	private:
		size_t _nextAgentId = 0;
		size_t GetNextId()
		{
			return _nextAgentId++;
		}

		std::map<size_t, ComponentId> _switchComponentTasks;

		void SwitchOpComponents()
		{
			for (auto task : _switchComponentTasks) {

				auto agentId = task.first;
				auto newOperationComponent = task.second;

				for (auto& c : _operComponents)
				{
					if (c->GetId() == newOperationComponent) {
						Agent & agent = _agents.find(agentId)->second;

						if(auto old = agent.opComponent.lock())
						{
							if (old != nullptr)
							{
								old->DeleteAgent(agentId);
							}
						}

						c->AddAgent(agentId);
						agent.opComponent = c;

						break;
					}
				}
			}

			_switchComponentTasks.clear();
		}

		float _currentTime = 0;

		std::shared_ptr<NavSystem> _navSystem;
		OnlineRecording _recording;
		bool _isRecording = false;

		std::map<size_t, FusionCrowd::Agent> _agents;
		std::map<ComponentId, std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::vector<std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::vector<std::shared_ptr<IOperationComponent>> _operComponents;

		GoalFactory _goalFactory;
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

	void Simulator::SetNavSystem(std::shared_ptr<NavSystem> navSystem)
	{
		pimpl->SetNavSystem(navSystem);
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
		float x, float y,
		ComponentId opId,
		ComponentId strategyId,
		ComponentId tacticId
	) {
		return pimpl->AddAgent(x, y, opId, strategyId, tacticId);
	}

	size_t Simulator::AddAgent(DirectX::SimpleMath::Vector2 pos)
	{
		return pimpl->AddAgent(pos);
	}

	void Simulator::SetAgentGoal(size_t agentId, DirectX::SimpleMath::Vector2 goalPos)
	{
		pimpl->SetAgentGoal(agentId, goalPos);
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
#pragma endregion
}
