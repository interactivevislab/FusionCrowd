#include "FsmStartegy.h"
#include "Navigation/AgentSpatialInfo.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	FsmStrategy::FsmStrategy(std::shared_ptr<Simulator> simulator, std::shared_ptr<NavSystem> navSystem)
		: _sim(simulator), _navSystem(navSystem)
	{
	}

	size_t FsmStrategy::AddMachine(Fsm::IFsm* machine)
	{
		size_t id = _nextMachineId++;

		_machines.insert({ id, std::unique_ptr<Fsm::IFsm>(machine) });
		return id;
	}

	void FsmStrategy::CreateGoToAction(Fsm::State duringState, float goalX, float goalY)
	{
		_gotoActions.insert({duringState, Vector2(goalX, goalY)});
	}

	void FsmStrategy::SetTickEvent(Fsm::Event fireEvt)
	{
		_tickEvent = fireEvt;
	}

	void FsmStrategy::CreateCloseToEvent(Fsm::Event fireEvt, float pointX, float pointY)
	{
		_closeToEvents.push_back({Vector2(pointX, pointY), fireEvt});
	}

	void FsmStrategy::AddAgent(size_t id, size_t machine_id)
	{
		_agentFsms.insert({id, { machine_id, _machines[machine_id]->GetInitialState() }});
	}

	void FsmStrategy::AddAgent(size_t id)
	{
		AddAgent(id, _nextMachineId - 1);
	}

	bool FsmStrategy::RemoveAgent(size_t id)
	{
		return false;
	}

	void FsmStrategy::SetAgentParams(size_t id, ModelAgentParams & params)
	{
		Fsm::AgentParams & aParams = static_cast<Fsm::AgentParams &>(params);

		_agentFsms[id] = { aParams.FsmId, _machines[aParams.FsmId]->GetInitialState() };
	}

	void FsmStrategy::Update(float timeStep)
	{
		for(auto & p: _agentFsms)
		{
			size_t agentId = p.first;
			size_t fsmId = p.second.fsmId;
			Fsm::State oldState = p.second.state;
			Fsm::State state = oldState;

			auto & agentInfo = _navSystem->GetSpatialInfo(agentId);

			state = _machines[fsmId]->Advance(oldState, _tickEvent);
			for (CloseToEventDesc & desc : _closeToEvents)
			{
				if((desc.target-agentInfo.pos).LengthSquared() < 0.2 * 0.2)
				{
					state = _machines[fsmId]->Advance(state, desc.eventToFire);
				}
			}

			if(state != oldState && _gotoActions.find(state) != _gotoActions.end())
			{
				_sim->SetAgentGoal(agentId, _gotoActions[state]);
			}

			p.second.state = state;
		}
	}
}