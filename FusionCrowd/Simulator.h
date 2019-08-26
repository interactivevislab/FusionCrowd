#pragma once

#include <vector>
#include <memory>

#include "Agent.h"
#include "Config.h"

#include "StrategyComponent/IStrategyComponent.h"
#include "StrategyComponent/Goal/Goal.h"
#include "TacticComponent/ITacticComponent.h"
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/NavSystem.h"

namespace FusionCrowd
{
	class NavMeshComponent;

	class FUSION_CROWD_API Simulator
	{
	public:
		Simulator();
		virtual ~Simulator();

		bool DoStep();

		bool SetOperationComponent(size_t agentId, std::string newOperationComponent);
		bool SetStrategyComponent(size_t agentId, std::string newStrategyComponent);

		NavSystem & GetNavSystem();

		float GetElapsedTime();

		// Need to remove following methods.
		// 1. Move to agent?
		size_t GetAgentCount() const;
		std::shared_ptr<Goal> GetAgentGoal(size_t agentId);

		// 2. Create simulator builder
		void AddOperComponent(std::shared_ptr<IOperationComponent> operComponent);
		void AddTacticComponent(std::shared_ptr<ITacticComponent> tacticComponent);
		void AddStrategyComponent(std::shared_ptr<IStrategyComponent> strategyComponent);
		void SetNavSystem(NavSystem && system);
		void InitSimulator(const char* navMeshPath);

		// 3. Should we create agent builder for that?
	    size_t AddAgent(
			float maxAngleVel,
			float radius,
			float prefSpeed,
			float maxSpeed,
			float maxAccel,
			DirectX::SimpleMath::Vector2 pos,
			std::shared_ptr<Goal> g
		);

		// 4. WTF is that?
		void UpdateNav(float x, float y);

	private:
		class SimulatorImpl;

		std::unique_ptr<SimulatorImpl> pimpl;
	};
}
