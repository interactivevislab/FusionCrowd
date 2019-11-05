#pragma once

#include "Export/IStrategyComponent.h"
#include "Export/ComponentId.h"
#include "Export/Fsm/IFsm.h"
#include "Export/Fsm/IStrategyConfigurator.h"

#include "Simulator.h"
#include "Navigation/NavSystem.h"

#include <map>
#include <memory>
#include <random>

#include <Math/Util.h>

namespace FusionCrowd
{
	class FsmStrategy : public IStrategyComponent, public Fsm::IStrategyConfigurator
	{
	public:
		FsmStrategy(
			std::shared_ptr<Simulator> simulator,
			std::shared_ptr<NavSystem> navSystem
		);

		size_t AddMachine(Fsm::IFsm* machine) override;
		void CreateGoToAction(const Fsm::State duringState, const Fsm::Point goal) override;
		void CreatePointReachEvent(const Fsm::Event fireEvt, const Fsm::Point point, const float radius) override;
		void CreateAnyPointReachEvent(const Fsm::Event fireEvt, const FCArray<Fsm::Point> & points, const float radius) override;

		void SetTickEvent(const Fsm::Event fireEvt) override;

		void AddAgent(size_t id) override;
		void AddAgent(size_t id, size_t machine_id);

		bool RemoveAgent(size_t id) override;
		void Update(float timeStep) override;

		void SetAgentParams(size_t id, ModelAgentParams & params) override;

		ComponentId GetId() override { return ComponentIds::FSM_ID; };


	private:
		struct CloseToEventDesc
		{
			DirectX::SimpleMath::Vector2 target;
			Fsm::Event eventToFire;
			float radiusSqr;
		};

		struct AgentFsmInfo
		{
			size_t fsmId;
			Fsm::State state;
		};

		size_t _nextMachineId = 0;

		Fsm::Event _tickEvent;

		std::default_random_engine _random_engine;
		std::shared_ptr<Simulator> _sim;
		std::shared_ptr<NavSystem> _navSystem;
		std::map<size_t, std::unique_ptr<Fsm::IFsm>> _machines;
		std::map<size_t, AgentFsmInfo> _agentFsms;

		std::vector<CloseToEventDesc> _closeToEvents;
		std::map<Fsm::State, DirectX::SimpleMath::Vector2> _gotoActions;
	};
}