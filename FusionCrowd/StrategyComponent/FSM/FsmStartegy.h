#pragma once

#include "Export/IStrategyComponent.h"
#include "Export/ComponentId.h"
#include "Export/Fsm/IFsm.h"
#include "Export/Fsm/IStrategyConfigurator.h"

#include "Simulator.h"
#include "Navigation/NavSystem.h"

#include <map>
#include <memory>

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
		void CreateGoToAction(Fsm::State duringState, float goalX, float goalY) override;
		void SetTickEvent(Fsm::Event fireEvt) override;
		void CreateCloseToEvent(Fsm::Event fireEvt, float pointX, float pointY) override;

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
		};

		struct AgentFsmInfo
		{
			size_t fsmId;
			Fsm::State state;
		};

		size_t _nextMachineId = 0;

		Fsm::Event _tickEvent;

		std::shared_ptr<Simulator> _sim;
		std::shared_ptr<NavSystem> _navSystem;
		std::map<size_t, std::unique_ptr<Fsm::IFsm>> _machines;
		std::map<size_t, AgentFsmInfo> _agentFsms;

		std::vector<CloseToEventDesc> _closeToEvents;
		std::map<Fsm::State, DirectX::SimpleMath::Vector2> _gotoActions;
	};
}