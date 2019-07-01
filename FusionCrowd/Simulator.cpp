#include "Simulator.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMeshComponent.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Simulator::Simulator(const char* navMeshPath)
	{
		_navMeshTactic = std::make_shared<NavMeshComponent>(*this, navMeshPath);

		AddTacticComponent(_navMeshTactic);

		_navSystem = std::make_shared<NavSystem>(_navMeshTactic);
	}

	bool Simulator::DoStep()
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

	NavSystem & Simulator::GetNavSystem()
	{
		return *_navSystem;
	}

	Agent & Simulator::GetById(size_t agentId)
	{
		return _agents[agentId];
	}

	size_t Simulator::AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos, std::shared_ptr<Goal> goal)
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

	bool Simulator::SetOperationComponent(size_t agentId, std::string newOperationComponent)
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

	bool Simulator::SetStrategyComponent(size_t agentId, std::string newStrategyComponent)
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

	void Simulator::AddOperComponent(std::shared_ptr<IOperationComponent> component)
	{
		_operComponents.push_back(component);
	}

	void Simulator::AddTacticComponent(std::shared_ptr<ITacticComponent> component)
	{
		_tacticComponents.push_back(component);
	}

	void Simulator::AddStrategyComponent(std::shared_ptr<IStrategyComponent> component)
	{
		_strategyComponents.push_back(component);
	}

	void Simulator::InitSimulator()
	{
		_navSystem->Init();
	}

	Simulator::~Simulator()
	{
	}

	void Simulator::UpdateNav(float x, float y)
	{
		DirectX::SimpleMath::Vector2 point1(-20.2780991, -22.8689995);
		_navMeshTactic->UpdateNavMesh(point1);
		for (int i = 0; i < _agents.size(); i++)
		{
			_navMeshTactic->AddAgent(_agents[i].id);
		}
	}
}
