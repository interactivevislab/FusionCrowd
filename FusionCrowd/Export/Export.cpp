#include "Export.h"

#include <memory>

#include "Math/Util.h"
#include "Simulator.h"

#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavSystem.h"

#include "TacticComponent/NavMeshComponent.h"

#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/ORCAComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "OperationComponent/PedVOComponent.h"

namespace FusionCrowd
{
	class SimulatorFacadeImpl : public ISimulatorFacade
	{
	public:
		SimulatorFacadeImpl(std::shared_ptr<Simulator> sim) : _sim(sim)
		{
		}

		void DoStep(float timeStep)
		{
			_sim->DoStep(timeStep);
		}

		OperationStatus SetAgentOp(size_t agentId, ComponentId opId)
		{
			_sim->SetOperationComponent(agentId, opId);

			return OperationStatus::OK;
		}

		OperationStatus SetAgentStrategy(size_t agentId, ComponentId strategyId)
		{
			_sim->SetStrategyComponent(agentId, strategyId);

			return OperationStatus::OK;
		}

		OperationStatus SetAgentGoal(size_t agentId, float x, float y)
		{
			_sim->SetAgentGoal(agentId, DirectX::SimpleMath::Vector2(x, y));

			return OperationStatus::OK;
		}

		bool GetAgents(FCArray<AgentInfo> & output)
		{
			return _sim->GetAgentsInfo(output);
		}

		size_t AddAgent(
			float x, float y,
			ComponentId opId,
			ComponentId strategyId
		)
		{
			return _sim->AddAgent(x, y, opId, strategyId);
		}

		OperationStatus RemoveAgent(size_t agentId)
		{
			return OperationStatus::NotImplemented;
		}

		IRecording & GetRecording()
		{
			return _sim->GetRecording();
		}

		size_t GetAgentCount()
		{
			return _sim->GetAgentCount();
		}
	private:
		std::shared_ptr<Simulator> _sim;
	};

	class BuilderImpl : public ISimulatorBuilder
	{
	public:
		BuilderImpl(): sim(std::make_shared<Simulator>())
		{
			impl = new SimulatorFacadeImpl(sim);
		}

		ISimulatorBuilder*  WithNavMesh(const char* path)
		{
			auto localizer = std::make_shared<NavMeshLocalizer>(path, true);
			navSystem = std::make_shared<NavSystem>(localizer);
			navSystem->Init();

			sim->UseNavSystem(navSystem);

			auto tactic = std::make_shared<FusionCrowd::NavMeshComponent>(sim, localizer);
			sim->AddTactic(tactic);

			return this;
		}

		ISimulatorBuilder* WithOp(ComponentId opId)
		{
			switch (opId)
			{
				case ComponentIds::KARAMOUZAS_ID:
					sim->AddOpModel(std::make_shared<Karamouzas::KaramouzasComponent>(navSystem));
					break;
				case ComponentIds::HELBING_ID:
					sim->AddOpModel(std::make_shared<Helbing::HelbingComponent>(navSystem));
					break;
				case ComponentIds::ORCA_ID:
					sim->AddOpModel(std::make_shared<ORCA::ORCAComponent>(navSystem));
					break;
				case ComponentIds::ZANLUNGO_ID:
					sim->AddOpModel(std::make_shared<Zanlungo::ZanlungoComponent>(navSystem));
					break;
				case ComponentIds::PEDVO_ID:
					sim->AddOpModel(std::make_shared<PedVO::PedVOComponent>(navSystem));
					break;
				default:
					break;
			}
			return this;
		}

		ISimulatorBuilder* WithStrategy(ComponentId strategyId)
		{

			return this;
		}

		ComponentId WithExternalStrategy(StrategyFactory externalStrategyFactory, IStrategyComponent ** outStrategy)
		{
			ComponentId newId = nextExternalStrategyId++;
			std::shared_ptr<IStrategyComponent> strat(externalStrategyFactory(impl, newId, outStrategy));

			sim->AddStrategy(strat);

			return newId;
		}

		ISimulatorFacade* Build()
		{
			return impl;
		}
	private:
		ComponentId nextExternalStrategyId = 900;

		SimulatorFacadeImpl* impl;

		std::shared_ptr<Simulator> sim;
		std::shared_ptr<NavSystem> navSystem;
	};

	ISimulatorBuilder* BuildSimulator()
	{
		return new BuilderImpl;
	}

	void BuilderDeleter(ISimulatorBuilder* builder)
	{
		delete builder;
	}

	void SimulatorFacadeDeleter(ISimulatorFacade* sim)
	{
		delete sim;
	}
}
