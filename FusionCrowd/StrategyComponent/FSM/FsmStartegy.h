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
#include <vector>

#include <Math/Util.h>

namespace FusionCrowd
{
	using AgentId = size_t;
	using MachineId = size_t;

	class FsmStrategy : public IStrategyComponent, public Fsm::IStrategyConfigurator
	{
	public:
		FsmStrategy(
			std::shared_ptr<Simulator> simulator,
			std::shared_ptr<NavSystem> navSystem
		);

		MachineId AddMachine(Fsm::IFsm* machine) override;
		void CreateGoToAction(const MachineId machineId, const Fsm::State duringState, const Fsm::Point goal) override;
		void CreateGoToAnyAction(const size_t machineId, const Fsm::State duringState, const FCArray<Fsm::Point> & goals) override;
		void CreatePointReachEvent(const MachineId machineId, const Fsm::Event fireEvt, const Fsm::Point point, const float radius) override;
		void CreateAnyPointReachEvent(const MachineId machineId, const Fsm::Event fireEvt, const FCArray<Fsm::Point> & points, const float radius) override;
		void CreateTimerEvent(const MachineId machineId, const Fsm::State duringState, const Fsm::Event fireEvt, const float minWaitTime, const float maxWaitTime) override;

		void SetTickEvent(const MachineId machineId, const Fsm::Event fireEvt) override;

		void AddAgent(AgentId id) override;
		void AddAgent(AgentId id, MachineId machine_id);

		bool RemoveAgent(AgentId id) override;
		void Update(float timeStep) override;

		void SetAgentParams(AgentId id, ModelAgentParams & params) override;

		ComponentId GetId() override { return ComponentIds::FSM_ID; };

	private:
		struct CloseToEventDesc
		{
			DirectX::SimpleMath::Vector2 target;
			Fsm::Event eventToFire;
			float radiusSqr;
		};

		struct TimerEventDesc
		{
			Fsm::State state;
			Fsm::Event eventToFire;
			float minTime;
			float maxTime;
		};

		struct ActiveTimer
		{
			Fsm::Event eventToFire;
			float timeLeft;
		};

		struct AgentFsmInfo
		{
			MachineId fsmId;
			Fsm::State state;
		};

		size_t _nextMachineId = 0;

		std::default_random_engine _random_engine;
		std::shared_ptr<Simulator> _sim;
		std::shared_ptr<NavSystem> _navSystem;
		std::map<MachineId, std::unique_ptr<Fsm::IFsm>> _machines;
		std::map<AgentId, AgentFsmInfo> _agentFsms;

		std::map<MachineId, std::vector<CloseToEventDesc>> _closeToEvents;

		std::map<MachineId, Fsm::Event> _tickEvents;
		std::map<MachineId, std::vector<TimerEventDesc>> _timerEvents;
		std::map<AgentId, std::vector<ActiveTimer>> _activeTimers;

		std::map<MachineId, std::map<Fsm::State, DirectX::SimpleMath::Vector2>> _gotoActions;
		std::map<MachineId, std::map<Fsm::State, std::vector<DirectX::SimpleMath::Vector2>>> _gotoAnyActions;
	};
}