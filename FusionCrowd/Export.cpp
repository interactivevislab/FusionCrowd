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
	static std::map<ComponentId, std::string> opComponentsMap = {
		{KARAMOUZAS_ID, "karamouzas"},
		{HELBING_ID,    "helbing"},
		{ORCA_ID,       "orca"},
		{ZANLUNGO_ID,   "zanlungo"},
		{PEDVO_ID,      "pedvo"}
	};

	static std::map<ComponentId, std::string> tacticComponentsMap = {
		{NAVMESH_ID, "NavMeshComponent"}
	};

	static std::map<ComponentId, std::string> strategyComponentsMap = {
	};

	class SimulatorFacadeImpl : public ISimulatorFacade
	{
	public:
		SimulatorFacadeImpl(std::shared_ptr<Simulator> sim) : _sim(sim)
		{
		}

		void DoStep()
		{
			_sim->DoStep();
		}

		OperationStatus SetAgentOp(size_t agentId, ComponentId opId)
		{
			_sim->SetOperationComponent(agentId, opComponentsMap[opId]);

			return OperationStatus::OK;
		}

		OperationStatus SetAgentStrategy(size_t agentId, ComponentId strategyId)
		{
			_sim->SetStrategyComponent(agentId, strategyComponentsMap[strategyId]);

			return OperationStatus::OK;
		}

		OperationStatus SetAgentGoal(size_t agentId, float x, float y)
		{
			_sim->SetAgentGoal(agentId, DirectX::SimpleMath::Vector2(x, y));

			return OperationStatus::OK;
		}

		FCArray<AgentInfo> GetAgents()
		{
			return _sim->GetAgentsInfo();
		}

		size_t AddAgent(
			float x, float y,
			ComponentId opId,
			ComponentId strategyId
		)
		{
			size_t agentId = _sim->AddAgent(DirectX::SimpleMath::Vector2(x, y));
			_sim->SetOperationComponent(agentId, opComponentsMap[opId]);
			_sim->SetTacticComponent(agentId, tacticComponentsMap[NAVMESH_ID]);
			_sim->SetOperationComponent(agentId, strategyComponentsMap[strategyId]);

			return agentId;
		}

		OperationStatus RemoveAgent(size_t agentId)
		{
			return OperationStatus::NotImplemented;
		}

		IRecording* GetRecording()
		{
			return _sim->GetNavSystem()->GetRecording();
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
				case KARAMOUZAS_ID:
					sim->AddOpModel(std::make_shared<Karamouzas::KaramouzasComponent>(navSystem));
					break;
				case HELBING_ID:
					sim->AddOpModel(std::make_shared<Helbing::HelbingComponent>(navSystem));
					break;
				case ORCA_ID:
					sim->AddOpModel(std::make_shared<ORCA::ORCAComponent>(navSystem));
					break;
				case ZANLUNGO_ID:
					sim->AddOpModel(std::make_shared<Zanlungo::ZanlungoComponent>(navSystem));
					break;
				case PEDVO_ID:
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

		ComponentId WithExternalStrategy(StrategyFactory externalStrategyFactory)
		{
			auto strat = std::shared_ptr<IStrategyComponent>(externalStrategyFactory(impl));
			ComponentId newId = nextExternalStrategyId++;

			sim->AddStrategy(strat);
			strategyComponentsMap.insert({newId, strat->GetName()});

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
