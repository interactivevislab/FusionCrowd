#include "Export.h"

#include <memory>

#include "Math/Util.h"
#include "Simulator.h"

#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavSystem.h"

#include "TacticComponent/NavMeshComponent.h"
#include "TacticComponent/NavGraphComponent.h"

#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/ORCAComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/GCFComponent.h"

#include "StrategyComponent/FSM/FsmStartegy.h"

#include "Group/FixedGridShape.h"
#include "Group/FreeGridShape.h"

using namespace DirectX::SimpleMath;

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

		OperationStatus SetAgentTactic(size_t agentId, ComponentId tacticID)
		{
			_sim->SetTacticComponent(agentId, tacticID);
			return OperationStatus::OK;
		}

		OperationStatus SetAgentStrategy(size_t agentId, ComponentId strategyId)
		{
			_sim->SetStrategyComponent(agentId, strategyId);

			return OperationStatus::OK;
		}

		OperationStatus SetAgentGoal(size_t agentId, float x, float y)
		{
			_sim->SetAgentGoal(agentId, Vector2(x, y));

			return OperationStatus::OK;
		}

		bool GetAgents(FCArray<AgentInfo> & output)
		{
			return _sim->GetAgentsInfo(output);
		}

		size_t AddAgent(
			float x, float y,
			ComponentId opId,
			ComponentId tacticId,
			ComponentId strategyId
		)
		{
			return _sim->AddAgent(x, y, opId, tacticId, strategyId);
		}

		size_t AddAgent(
			float x, float y, float radius,
			ComponentId opId,
			ComponentId tacticId,
			ComponentId strategyId
		)
		{
			AgentSpatialInfo info;
			info.pos = Vector2(x, y);
			info.radius = radius;

			return _sim->AddAgent(info, opId, tacticId, strategyId);
		}

		OperationStatus RemoveAgent(size_t agentId)
		{
			return _sim->RemoveAgent(agentId);
		}

		IRecording & GetRecording()
		{
			return _sim->GetRecording();
		}

		void SetIsRecording(bool isRecording) {
			_sim->SetIsRecording(isRecording);
		}

		size_t GetAgentCount()
		{
			return _sim->GetAgentCount();
		}

		void SetAgentStrategyParam(size_t agentId, ComponentId strategyId, ModelAgentParams & params)
		{
			_sim->SetAgentStrategyParam(agentId, strategyId, params);
		}

		IStrategyComponent* GetStrategy(ComponentId strategyId) const
		{
			return _sim->GetStrategy(strategyId);
		}

		INavMeshPublic* GetNavMesh() const
		{
			return _sim->GetNavSystem()->GetPublicNavMesh();
		}

		INavSystemPublic* GetNavSystem() const
		{
			return _sim->GetNavSystem();
		}

		size_t AddGridGroup(float x, float y, size_t agetsInRow, float interAgtDist)
		{
			auto grid = std::make_unique<FixedGridShape>(agetsInRow, interAgtDist);

			return _sim->AddGroup(std::move(grid), DirectX::SimpleMath::Vector2(x, y));
		}

		size_t AddFreeGridGroup(float x, float y, size_t agetsInRow, float interAgtDist)
		{
			auto freeGrid = std::make_unique<FreeGridShape>(agetsInRow, interAgtDist);

			return _sim->AddGroup(std::move(freeGrid), DirectX::SimpleMath::Vector2(x, y));
		}

		void RemoveAgentFromGroup(size_t agentId, size_t groupId)
		{
			_sim->RemoveAgentFromGroup(agentId, groupId);
		}

		void SetGroupGoal(size_t groupId, float goalX, float goalY)
		{
			_sim->SetGroupGoal(groupId, DirectX::SimpleMath::Vector2(goalX, goalY));
		}

		void AddAgentToGroup(size_t agentId, size_t groupId)
		{
			_sim->AddAgentToGroup(agentId, groupId);
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
			navSystem = std::make_shared<NavSystem>();

			sim->UseNavSystem(navSystem);
		}

		ISimulatorBuilder*  WithNavMesh(const char* path)
		{
			auto localizer = std::make_shared<NavMeshLocalizer>(path, true);
			navSystem->SetNavMesh(localizer);

			auto spatial_query = std::make_shared<NavMeshSpatialQuery>(localizer);
			auto tactic = std::make_shared<FusionCrowd::NavMeshComponent>(sim, localizer, spatial_query);
			sim->AddTactic(tactic);

			return this;
		}

		ISimulatorBuilder* WithNavGraph(const char* path)
		{
			std::ifstream f(path);

			if (!f.is_open())
			{
				throw std::ios_base::failure("Can't load navgraph");
			}

			navSystem->SetNavGraph(NavGraph::LoadFromStream(f));

			auto tactic = std::make_shared<FusionCrowd::NavGraphComponent>(sim, navSystem);
			sim->AddTactic(tactic);

			return this;
		}

		ISimulatorBuilder* WithNavGraph(FCArray<Export::NavGraphNode> & nodesArray, FCArray<Export::NavGraphEdge> & edgesArray)
		{
			std::vector<NavGraphNode> nodes;
			for(auto& n : nodesArray)
			{
				nodes.push_back({
					n.id, DirectX::SimpleMath::Vector2(n.x, n.y)
				});
			}

			std::vector<NavGraphEdge> edges;
			size_t eId = 0;
			for(auto& e : edgesArray)
			{
				edges.push_back({
					eId++,
					e.nodeFrom,
					e.nodeTo,
					e.weight,
					e.width
				});
			}

			navSystem->SetNavGraph(std::make_unique<NavGraph>(nodes, edges));

			auto tactic = std::make_shared<FusionCrowd::NavGraphComponent>(sim, navSystem);
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
				case ComponentIds::GCF_ID:
					sim->AddOpModel(std::make_shared<GCF::GCFComponent>(navSystem));
					break;
				default:
					break;
			}
			return this;
		}

		ISimulatorBuilder* WithStrategy(ComponentId strategyId)
		{
			switch (strategyId)
			{
				case ComponentIds::FSM_ID:
					sim->AddStrategy(std::make_shared<FsmStrategy>(sim, navSystem));
					break;
				default:
					break;
			}
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
			navSystem->Init();

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
