#include "Simulator.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMeshComponent.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
#pragma region Simulator
	class Simulator::SimulatorImpl
	{
	public:
		SimulatorImpl()
		{
		}

		~SimulatorImpl() = default;

		/*
		void Initialize(Simulator & simulator, const char* navMeshPath)
		{
			_navMeshTactic = std::make_shared<NavMeshComponent>(simulator, navMeshPath);

			AddTacticComponent(_navMeshTactic);

			_navSystem = NavSystem();
			_navSystem.SetNavComponent(*_navMeshTactic);

			_navSystem.Init();
		}
		*/

		bool DoStep()
		{
			const float timeStep = 0.1f;

			_currentTime += timeStep;

			for (auto strategy : _strategyComponents)
			{
				strategy->Update(timeStep);
			}

			for (auto tactic : _tacticComponents)
			{
				tactic->Update(timeStep);
			}

			for (auto oper : _operComponents)
			{
				oper->Update(timeStep);
			}

			_navSystem->Update(timeStep);

			return true;
		}

		size_t GetAgentCount() const { return _agents.size(); }

		std::shared_ptr<NavSystem> GetNavSystem()
		{
			return _navSystem;
		}

		std::shared_ptr<Goal> GetAgentGoal(size_t agentId) {
			return _agents[agentId].currentGoal;
		}

	    size_t AddAgent(
			float maxAngleVel,
			float radius,
			float prefSpeed,
			float maxSpeed,
			float maxAccel,
			DirectX::SimpleMath::Vector2 pos,
			std::shared_ptr<Goal> goal
		)
		{
			size_t id = _agents.size();

			AgentSpatialInfo info;
			info.id = id;
			info.pos = pos;
			info.radius = radius;
			info.maxAngVel = maxAngleVel;
			info.prefSpeed = prefSpeed;
			info.maxSpeed = maxSpeed;
			info.maxAccel = maxAccel;

			_navSystem->AddAgent(info);
			Agent a(id);
			a.currentGoal = goal;
			_agents.push_back(a);

			//TEMPORARY
			_navMeshTactic->AddAgent(id);

			return id;
		}

		bool SetOperationComponent(size_t agentId, std::string newOperationComponent)
		{
			for(auto& c : _operComponents)
			{
				if(c->GetName() == newOperationComponent) {
					std::shared_ptr<IOperationComponent> old = _agents[agentId].opComponent;
					if(old != nullptr)
					{
						old->DeleteAgent(agentId);
					}

					c->AddAgent(agentId);
					_agents[agentId].opComponent = c;

					return true;
				}
			}
			return false;
		}

		bool SetStrategyComponent(size_t agentId, std::string newStrategyComponent)
		{
			for(auto& c : _strategyComponents)
			{
				if(c->GetName() == newStrategyComponent) {
					std::shared_ptr<IStrategyComponent>& old = _agents[agentId].stratComponent;
					if(old != nullptr)
					{
						old->RemoveAgent(agentId);
					}

					c->AddAgent(agentId);
					_agents[agentId].stratComponent = c;

					return true;
				}
			}
			return false;
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
			_strategyComponents.push_back(strategyComponent);
		}

		void InitSimulator() {
		}

		void UpdateNav(float x, float y)
		{
			DirectX::SimpleMath::Vector2 point1(-20.2780991, -22.8689995);
			_navMeshTactic->UpdateNavMesh(point1);
			for (int i = 0; i < _agents.size(); i++)
			{
				_navMeshTactic->AddAgent(_agents[i].id);
			}
		}

		float GetElapsedTime()
		{
			return _currentTime;
		}

		void SetNavSystem(std::shared_ptr<NavSystem> navSystem)
		{
			_navSystem = navSystem;
		}

		// TEMPORARY SOLUTION
		std::shared_ptr<NavMeshComponent> _navMeshTactic;
	private:
		size_t GetNextId() const { return GetAgentCount(); }

		float _currentTime = 0;

		std::shared_ptr<NavSystem> _navSystem;

		std::vector<FusionCrowd::Agent> _agents;
		std::vector<std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::vector<std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::vector<std::shared_ptr<IOperationComponent>> _operComponents;
	};

	Simulator::~Simulator() { };

	Simulator::Simulator() : pimpl(std::make_unique<SimulatorImpl>())
	{
	}

	Simulator::Simulator(Simulator && other)
	{
		if(this == &other)
			return;

		this->pimpl = std::move(other.pimpl);
	}

	Simulator& Simulator::operator=(Simulator && other)
	{
		if(this == &other)
		{
			return *this;
		}

		this->pimpl = std::move(other.pimpl);

		return *this;
	}

	bool Simulator::DoStep()
	{
		return pimpl->DoStep();
	}

	size_t Simulator::GetAgentCount() const
	{
		return pimpl->GetAgentCount();
	}

	std::shared_ptr<NavSystem> Simulator::GetNavSystem()
	{
		return pimpl->GetNavSystem();
	}

	std::shared_ptr<Goal> Simulator::GetAgentGoal(size_t agentId) {
		return pimpl->GetAgentGoal(agentId);
	}

	size_t Simulator::AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos, std::shared_ptr<Goal> goal)
	{
		return pimpl->AddAgent(maxAngleVel, radius, prefSpeed, maxSpeed, maxAccel, pos, goal);
	}

	bool Simulator::SetOperationComponent(size_t agentId, std::string newOperationComponent)
	{
		return pimpl->SetOperationComponent(agentId, newOperationComponent);
	}

	bool Simulator::SetStrategyComponent(size_t agentId, std::string newStrategyComponent)
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

	void Simulator::UpdateNav(float x, float y)
	{
		pimpl->UpdateNav(x, y);
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
#pragma endregion
}
