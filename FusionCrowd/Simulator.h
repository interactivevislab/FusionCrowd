#pragma once

#include <vector>
#include <memory>

#include "Agent.h"

#include "Export/ComponentId.h"
#include "Export/Export.h"
#include "Export/FCArray.h"
#include "Export/IStrategyComponent.h"

#include "StrategyComponent/Goal/Goal.h"
#include "TacticComponent/ITacticComponent.h"
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/NavSystem.h"

#include "Util/spimpl.h"
#include "Math/Util.h"

#include "Group/IGroup.h"

#include <memory>

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

		void SetAgentStrategyParam(size_t agentId, ComponentId strategyId, ModelAgentParams & params);
		IStrategyComponent* GetStrategy(ComponentId strategyId) const;

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		IRecording & GetRecording();
		void SetIsRecording(bool isRecording);

		float GetElapsedTime();

		size_t GetAgentCount() const;
		const Goal & GetAgentGoal(size_t agentId) const;

		bool UpdateAgentParams(AgentParams params);
		bool UpdateNeighbourSearchShape(size_t agentId, Cone cone);
		bool UpdateNeighbourSearchShape(size_t agentId, Disk disk);

		OperationStatus RemoveAgent(size_t agentId);

		OperationStatus RemoveGroup(size_t groupId);

		size_t AddAgent(DirectX::SimpleMath::Vector2 pos);

		size_t AddAgent(
			DirectX::SimpleMath::Vector2 pos,
			ComponentId opId,
			ComponentId strategyId,
			ComponentId tacticId
		);

		size_t AddAgent(
			AgentSpatialInfo props,
			ComponentId opId,
			ComponentId tacticId,
			ComponentId strategyId
		);

		void SetAgentGoal(size_t agentId, Goal && goal);
		Agent & GetAgent(size_t id);

		size_t AddGridGroup(DirectX::SimpleMath::Vector2 origin, size_t agentsInRow, float interAgentDistance);
		size_t AddGuidedGroup(size_t leaderId);

		void SetGroupGoal(size_t groupId, DirectX::SimpleMath::Vector2 goalPos);

		void AddAgentToGroup(size_t agentId, size_t groupId);
		void RemoveAgentFromGroup(size_t agentId, size_t groupId);

		IGroup* GetGroup(size_t groupId);

		FCArray<AgentInfo> GetAgentsInfo();

		// Must: output.len >= GetAgentCount();
		bool GetAgentsInfo(FCArray<AgentInfo> & output);

		NavSystem* GetNavSystem() const;
		GoalFactory & GetGoalFactory();
	private:
		class SimulatorImpl;

		spimpl::unique_impl_ptr<SimulatorImpl> pimpl;
	};
}
