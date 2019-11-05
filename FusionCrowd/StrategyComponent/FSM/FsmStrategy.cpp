#include "FsmStartegy.h"
#include "Navigation/AgentSpatialInfo.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	FsmStrategy::FsmStrategy(std::shared_ptr<Simulator> simulator, std::shared_ptr<NavSystem> navSystem)
		: _sim(simulator), _navSystem(navSystem), _random_engine(std::random_device()())
	{
	}

	size_t FsmStrategy::AddMachine(Fsm::IFsm* machine)
	{
		size_t id = _nextMachineId++;

		_machines.insert({ id, std::unique_ptr<Fsm::IFsm>(machine) });
		return id;
	}

	void FsmStrategy::CreateGoToAction(const Fsm::State duringState, const Fsm::Point goal)
	{
		_gotoActions.insert({ duringState, Vector2(goal.x, goal.y) });
	}

	void FsmStrategy::SetTickEvent(const Fsm::Event fireEvt)
	{
		_tickEvent = fireEvt;
	}

	void FsmStrategy::CreatePointReachEvent(const Fsm::Event fireEvt, const Fsm::Point point, const float radius)
	{
		_closeToEvents.push_back({Vector2(point.x, point.y), fireEvt, radius * radius});
	}

	void FsmStrategy::CreateAnyPointReachEvent(const Fsm::Event fireEvt, const FCArray<Fsm::Point> & points, const float radius)
	{
		for(const Fsm::Point & p : points)
		{
			_closeToEvents.push_back({Vector2(p.x, p.y), fireEvt, radius * radius});
		}
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
				if((desc.target-agentInfo.pos).LengthSquared() < desc.radiusSqr)
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