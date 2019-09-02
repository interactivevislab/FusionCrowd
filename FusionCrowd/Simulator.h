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
		~Simulator();

		Simulator & AddOpModel(std::shared_ptr<IOperationComponent> operComponent);
		Simulator & AddTactic(std::shared_ptr<ITacticComponent> tacticComponent);
		Simulator & AddStrategy(std::shared_ptr<IStrategyComponent> strategyComponent);
		Simulator & UseNavSystem(std::shared_ptr<NavSystem> system);

		Simulator(Simulator &&);
		Simulator& operator=(Simulator &&);

		bool DoStep();

		bool SetOperationComponent(size_t agentId, std::string newOperationComponent);
		bool SetTacticComponent(size_t agentId, std::string newTactic);
		bool SetStrategyComponent(size_t agentId, std::string newStrategyComponent);
		void SetNavSystem(std::shared_ptr<NavSystem> navSystem);

		std::shared_ptr<NavSystem> GetNavSystem();

		float GetElapsedTime();

		// Need to remove following methods.
		// 1. Move to agent?
		size_t GetAgentCount() const;
		std::shared_ptr<Goal> GetAgentGoal(size_t agentId);

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

	private:
		class SimulatorImpl;

		std::unique_ptr<SimulatorImpl> pimpl;
	};
}
