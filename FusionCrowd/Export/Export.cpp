#include "Export.h"

#include <memory>

#include "Math/Util.h"
#include "Simulator.h"

#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavSystem.h"

#include "TacticComponent/NavMesh/NavMeshComponent.h"
#include "TacticComponent/NavGraph/NavGraphComponent.h"
#include "TacticComponent/ExternalControl.h"

#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/ORCAComponent.h"
#include "OperationComponent/ZanlungoComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/GCFComponent.h"
#include "OperationComponent/BicycleComponent.h"
#include "OperationComponent/StrictComponent.h"

#include "StrategyComponent/FSM/FsmStartegy.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class SimulatorFacadeImpl : public ISimulatorFacade
	{
	public:
		SimulatorFacadeImpl(std::shared_ptr<Simulator> sim) : _sim(sim)
		{}

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

		OperationStatus SetAgentGoal(size_t agentId, Point p)
		{
			_sim->SetAgentGoal(agentId, _sim->GetGoalFactory().CreatePointGoal(p));

			return OperationStatus::OK;
		}

		OperationStatus SetAgentGoal(size_t agentId, Disk d)
		{
			_sim->SetAgentGoal(agentId, _sim->GetGoalFactory().CreateDiscGoal(d));

			return OperationStatus::OK;
		}

		OperationStatus SetAgentGoal(size_t agentId, Rect r)
		{
			_sim->SetAgentGoal(agentId, _sim->GetGoalFactory().CreateRectGoal(r));

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
			return _sim->AddAgent(Vector2(x, y), opId, tacticId, strategyId);
		}

		size_t AddAgent(
			float x, float y, float radius, float preferedVelocity,
			ComponentId opId,
			ComponentId tacticId,
			ComponentId strategyId
		)
		{
			AgentSpatialInfo info;
			info.radius = radius;
			info.maxSpeed = preferedVelocity * 1.5f;
			info.prefSpeed = preferedVelocity;
			info.SetPos(Vector2(x, y));

			return _sim->AddAgent(std::move(info), opId, tacticId, strategyId);
		}


		bool UpdateAgent(AgentParams params) {
			return _sim->UpdateAgentParams(params);
		}

		bool UpdateNeighbourSearchShape(size_t agentId, Disk disk)
		{
			return _sim->UpdateNeighbourSearchShape(agentId, disk);
		}

		bool UpdateNeighbourSearchShape(size_t agentId, Cone cone)
		{
			return _sim->UpdateNeighbourSearchShape(agentId, cone);
		}

		OperationStatus RemoveAgent(size_t agentId)
		{
			return _sim->RemoveAgent(agentId);
		}

		OperationStatus RemoveGroup(size_t groupId)
		{
			return _sim->RemoveGroup(groupId);
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

		size_t AddGridGroup(float x, float y, size_t agentsInRow, float interAgtDist)
		{
			return _sim->AddGridGroup(Vector2(x, y), agentsInRow, interAgtDist);
		}

		size_t AddGuidedGroup(size_t leaderId)
		{
			return _sim->AddGuidedGroup(leaderId);
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

		size_t GetGroupDummyAgent(size_t groupId)
		{
			return _sim->GetGroup(groupId)->GetDummyId();
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

		ISimulatorBuilder* WithExternal(IExternalControllInterface*& returned_controll)
		{
			auto tactic = std::make_shared<FusionCrowd::ExternalControl>(sim);
			sim->AddTactic(tactic);
			returned_controll = tactic.get();

			atLeastOneTactic = true;

			return this;
		}

		ISimulatorBuilder* WithNavMesh(const char* path)
		{
			auto localizer = std::make_shared<NavMeshLocalizer>(path, true);
			navSystem->SetNavMesh(localizer);

			auto spatial_query = std::make_shared<NavMeshSpatialQuery>(localizer);
			auto tactic = std::make_shared<FusionCrowd::NavMeshComponent>(sim, localizer, spatial_query);
			sim->AddTactic(tactic);

			atLeastOneTactic = true;

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

			atLeastOneTactic = true;

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

			atLeastOneTactic = true;

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
				case ComponentIds::BICYCLE:
					sim->AddOpModel(std::make_shared<Bicycle::BicycleComponent>(navSystem));
					break;
				case ComponentIds::STRICT_ID:
					sim->AddOpModel(std::make_shared<StrictComp::StrictComponent>(navSystem));
					break;
				default:
					break;
			}

			atLeastOneOperational = true;

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
			if(!atLeastOneOperational)
				throw "At least one op component has to be defined";

			if(!atLeastOneTactic)
				throw "At least one tactic component has to be defined";

			navSystem->Init();

			return impl;
		}
	private:
		ComponentId nextExternalStrategyId = 900;

		SimulatorFacadeImpl* impl;

		bool atLeastOneTactic      = false;
		bool atLeastOneOperational = false;

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
