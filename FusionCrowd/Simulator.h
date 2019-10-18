#pragma once

#include <vector>
#include <memory>

#include "Agent.h"

#include "Util/spimpl.h"
#include "Export/FCArray.h"
#include "Math/Util.h"
#include "Export/IStrategyComponent.h"
#include "StrategyComponent/Goal/Goal.h"
#include "TacticComponent/ITacticComponent.h"
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/NavSystem.h"
#include "Export/Export.h"
#include "Export/ComponentId.h"

namespace FusionCrowd
{
	class NavMeshComponent;

	class Simulator
	{
	public:
		Simulator();

		Simulator & AddOpModel(std::shared_ptr<IOperationComponent> operComponent);
		Simulator & AddTactic(std::shared_ptr<ITacticComponent> tacticComponent);
		Simulator & AddStrategy(std::shared_ptr<IStrategyComponent> strategyComponent);
		Simulator & UseNavSystem(std::shared_ptr<NavSystem> system);

		bool DoStep(float timeStep);

		bool SetOperationComponent(size_t agentId, ComponentId newOperationComponent);
		bool SetTacticComponent(size_t agentId, ComponentId newTactic);
		bool SetStrategyComponent(size_t agentId, ComponentId newStrategyComponent);
		void SetNavSystem(std::shared_ptr<NavSystem> navSystem);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		IRecording & GetRecording();
		void SetIsRecording(bool isRecording);

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

		size_t AddAgent(DirectX::SimpleMath::Vector2 pos);
		size_t AddAgent(
			float x, float y,
			ComponentId opId,
			ComponentId strategyId
		);

		void SetAgentGoal(size_t agentId, DirectX::SimpleMath::Vector2 goalPos);

		Agent & GetAgent(size_t id);

		FCArray<AgentInfo> GetAgentsInfo();

		// Must: output.len >= GetAgentCount();
		bool GetAgentsInfo(FCArray<AgentInfo> & output);
	private:
		class SimulatorImpl;

		spimpl::unique_impl_ptr<SimulatorImpl> pimpl;
	};
}
