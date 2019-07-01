#include "Simulator.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMeshComponent.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class Simulator::SimulatorImpl
	{
	public:
		SimulatorImpl()
		{
		}

		~SimulatorImpl() = default;

		void Initialize(Simulator & simulator, const char* navMeshPath)
		{
			_navMeshTactic = std::make_shared<NavMeshComponent>(simulator, navMeshPath);

			AddTacticComponent(_navMeshTactic);

			_navSystem = std::make_shared<NavSystem>(_navMeshTactic);
		}

		bool DoStep()
		{
			const float timeStep = 0.1f;
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
		Agent & GetById(size_t agentId)
		{
			return _agents[agentId];
		}

		NavSystem & GetNavSystem()
		{
			return *_navSystem;
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
			for(std::shared_ptr<IOperationComponent> c : _operComponents)
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
			for(std::shared_ptr<IStrategyComponent> c : _strategyComponents)
			{
				if(c->GetName() == newStrategyComponent) {
					std::shared_ptr<IStrategyComponent> old = _agents[agentId].stratComponent;
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
			_navSystem->Init();
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

		// TEMPORARY SOLUTION
		std::shared_ptr<NavMeshComponent> _navMeshTactic;
	private:

		size_t GetNextId() const { return GetAgentCount(); }

		std::shared_ptr<NavSystem> _navSystem;

		std::vector<FusionCrowd::Agent> _agents;
		std::vector<std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::vector<std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::vector<std::shared_ptr<IOperationComponent>> _operComponents;
	};

	Simulator::~Simulator() = default;

	Simulator::Simulator(const char* navMeshPath)
		: pimpl(std::make_unique<SimulatorImpl>())
	{
		pimpl->Initialize(*this, navMeshPath);
	}

	bool Simulator::DoStep()
	{
		return pimpl->DoStep();
	}

	size_t Simulator::GetAgentCount() const
	{
		return pimpl->GetAgentCount();
	}

	NavSystem & Simulator::GetNavSystem()
	{
		return pimpl->GetNavSystem();
	}

	Agent & Simulator::GetById(size_t agentId)
	{
		return pimpl->GetById(agentId);
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

	void Simulator::AddOperComponent(std::shared_ptr<IOperationComponent> component)
	{
		pimpl->AddOperComponent(component);
	}

	void Simulator::AddTacticComponent(std::shared_ptr<ITacticComponent> component)
	{
		pimpl->AddTacticComponent(component);
	}

	void Simulator::AddStrategyComponent(std::shared_ptr<IStrategyComponent> component)
	{
		pimpl->AddStrategyComponent(component);
	}

	void Simulator::InitSimulator()
	{
		pimpl->InitSimulator();
	}

	void Simulator::UpdateNav(float x, float y)
	{
		pimpl->UpdateNav(x, y);
	}
}
