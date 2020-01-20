#pragma once

#include "Export/FCArray.h"
#include "Export/IRecording.h"
#include "Export/IStrategyComponent.h"
#include "Export/INavMeshPublic.h"
#include "Export/INavSystemPublic.h"
#include "Export/ComponentId.h"
#include "Export/ModelAgentParams.h"

namespace FusionCrowd
{
	class IRecording;

	extern "C"
	{
		struct FUSION_CROWD_API AgentInfo
		{
			size_t id;

			float posX, posY;
			float velX, velY;
			float orientX, orientY;
			float radius;

			ComponentId opCompId;
			ComponentId tacticCompId;
			ComponentId stratCompId;

			float goalX, goalY;
		};

		enum FUSION_CROWD_API OperationStatus
		{
			OK,
			InvalidArgument,
			InternalError,
			NotImplemented,
		};

		class FUSION_CROWD_API ISimulatorFacade
		{
		public:
			virtual void DoStep(float timeStep = 0.1f) = 0;

			virtual OperationStatus SetAgentOp(size_t agentId, ComponentId opId) = 0;
			virtual OperationStatus SetAgentStrategy(size_t agentId, ComponentId strategyId) = 0;
			virtual void SetAgentStrategyParam(size_t agentId, ComponentId strategyId, ModelAgentParams & params) = 0;
			virtual OperationStatus SetAgentGoal(size_t agentId, float x, float y) = 0;

			virtual size_t GetAgentCount() = 0;

			virtual bool GetAgents(FCArray<AgentInfo> & output) = 0;

			virtual size_t AddAgent(
				float x, float y,
				ComponentId opId,
				ComponentId strategyId
			) = 0;
			virtual OperationStatus RemoveAgent(size_t agentId) = 0;

			virtual IRecording & GetRecording() = 0;

			virtual IStrategyComponent* GetStrategy(ComponentId strategyId) const = 0;
			virtual void SetIsRecording(bool isRecording) = 0;

			virtual INavMeshPublic* GetNavMesh() const = 0;
			virtual INavSystemPublic* GetNavSystem() const = 0;
		};

		/*
		 * To be used with std::shared_ptr:
		 * std::shared_ptr<ISimulatorFacade> sim(builder->Build(), SimulatorFacadeDeleter);
		 */
		FUSION_CROWD_API void SimulatorFacadeDeleter(ISimulatorFacade* sim);


		// Params: simulator, assignedId, out result to be used outside of the simulator
		FUSION_CROWD_API typedef IStrategyComponent* (*StrategyFactory)(ISimulatorFacade *, ComponentId, IStrategyComponent **);

		class FUSION_CROWD_API ISimulatorBuilder
		{
		public:
			virtual ISimulatorBuilder* WithNavMesh(const char* path) = 0;
			virtual ISimulatorBuilder* WithOp(ComponentId opId) = 0;
			virtual ISimulatorBuilder* WithStrategy(ComponentId strategyId) = 0;

			virtual ComponentId WithExternalStrategy(StrategyFactory externalStrategyFactory, IStrategyComponent ** outStrategy) = 0;

			virtual ISimulatorFacade* Build() = 0;
		};

		/*
		 * To be used with std::shared_ptr:
		 * std::shared_ptr<ISimulatorBuilder> b(BuildSimulator(), BuilderDeleter);
		 */
		FUSION_CROWD_API ISimulatorBuilder* BuildSimulator();
		FUSION_CROWD_API void BuilderDeleter(ISimulatorBuilder* builder);

	}
}
