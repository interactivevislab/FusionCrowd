#pragma once

#include <vector>
#include <memory>

#include "Agent.h"
#include "Config.h"

#include "StrategyComponent/IStrategyComponent.h"
#include "StrategyComponent/Goal/PointGoal.h"
#include "TacticComponent/NavMeshComponent.h"
#include "TacticComponent/ITacticComponent.h"
#include "OperationComponent/IOperationComponent.h"

#include "Navigation/NavSystem.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API Simulator
	{
	public:
		Simulator(const char* navMeshPath);
		~Simulator();

		bool DoStep();

		size_t GetAgentCount() const { return _agents.size(); }
		Agent & GetById(size_t agentId);
		NavSystem & GetNavSystem();

	    size_t AddAgent(
			float maxAngleVel,
			float radius,
			float prefSpeed,
			float maxSpeed,
			float maxAccel,
			DirectX::SimpleMath::Vector2 pos,
			std::shared_ptr<Goal> g
		);

		bool SetOperationComponent(size_t agentId, std::string newOperationComponent);
		bool SetStrategyComponent(size_t agentId, std::string newStrategyComponent);

		void AddOperComponent(std::shared_ptr<IOperationComponent> operComponent);
		void AddTacticComponent(std::shared_ptr<ITacticComponent> tacticComponent);
		void AddStrategyComponent(std::shared_ptr<IStrategyComponent> strategyComponent);

		void InitSimulator();
	private:
		size_t GetNextId() const { return GetAgentCount(); }

		NavSystem * _navSystem;

		// TEMPORARY SOLUTION
		std::shared_ptr<NavMeshComponent> _navMeshTactic;

		std::vector<FusionCrowd::Agent> _agents;
		std::vector<std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::vector<std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::vector<std::shared_ptr<IOperationComponent>> _operComponents;
	};
}
