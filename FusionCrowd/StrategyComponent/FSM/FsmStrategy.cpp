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

	void FsmStrategy::CreateGoToAction(const MachineId machineId, const Fsm::State duringState, const Fsm::Point goal)
	{
		if(_gotoActions.find(machineId) == _gotoActions.end())
		{
			_gotoActions.insert({machineId, { }});
		}

		_gotoActions[machineId].insert({ duringState, Vector2(goal.x, goal.y) });
	}

	void FsmStrategy::CreateGoToAnyAction(const size_t machineId, const Fsm::State duringState, const FCArray<Fsm::Point> & goals)
	{
		if(_gotoAnyActions.find(machineId) == _gotoAnyActions.end())
		{
			_gotoAnyActions.insert({machineId, { }});
		}

		std::vector<Vector2> goalsVector;
		for(auto & p : goals)
		{
			goalsVector.push_back(Vector2(p.x, p.y));
		}

		_gotoAnyActions[machineId].insert({ duringState, goalsVector});
	}

	void FsmStrategy::SetTickEvent(const MachineId machineId, const Fsm::Event fireEvt)
	{
		_tickEvents.insert({ machineId, fireEvt });
	}

	void FsmStrategy::CreatePointReachEvent(const MachineId machineId, const Fsm::Event fireEvt, const Fsm::Point point, const float radius)
	{
		if(_closeToEvents.find(machineId) == _closeToEvents.end())
		{
			_closeToEvents.insert({ machineId, { } });
		}

		_closeToEvents[machineId].push_back({Vector2(point.x, point.y), fireEvt, radius * radius});
	}

	void FsmStrategy::CreateAnyPointReachEvent(const MachineId machineId, const Fsm::Event fireEvt, const FCArray<Fsm::Point> & points, const float radius)
	{
		if(_closeToEvents.find(machineId) == _closeToEvents.end())
		{
			_closeToEvents.insert({ machineId, { } });
		}

		for(const Fsm::Point & p : points)
		{
			_closeToEvents[machineId].push_back({Vector2(p.x, p.y), fireEvt, radius * radius});
		}
	}

	void FsmStrategy::CreateTimerEvent(const MachineId machineId, const Fsm::State duringState, const Fsm::Event fireEvt, const float minWaitTime, const float maxWaitTime)
	{
		if(_timerEvents.find(machineId) == _timerEvents.end())
		{
			_timerEvents.insert({ machineId, { }});
		}

		_timerEvents[machineId].push_back({duringState, fireEvt, minWaitTime, maxWaitTime});
	}

	void FsmStrategy::AddAgent(AgentId id, MachineId machine_id)
	{
		_agentFsms.insert({id, { machine_id, _machines[machine_id]->GetInitialState() }});
	}

	void FsmStrategy::AddAgent(AgentId id)
	{
		AddAgent(id, _nextMachineId - 1);
	}

	bool FsmStrategy::RemoveAgent(AgentId id)
	{
		return false;
	}

	void FsmStrategy::SetAgentParams(AgentId id, ModelAgentParams & params)
	{
		Fsm::AgentParams & aParams = static_cast<Fsm::AgentParams &>(params);

		_agentFsms[id] = { aParams.FsmId, _machines[aParams.FsmId]->GetInitialState() };
	}

	void FsmStrategy::Update(float timeStep)
	{
		auto & goalFactory = _sim->GetGoalFactory();
		for(auto & p: _agentFsms)
		{
			AgentId agentId = p.first;
			MachineId fsmId = p.second.fsmId;
			Fsm::State state = p.second.state;
			Fsm::State oldState = state;

			auto & agentInfo = _navSystem->GetSpatialInfo(agentId);

			state = _machines[fsmId]->Advance(state, _tickEvents[fsmId]);

			if(state == oldState)
			{
				for (auto & desc : _closeToEvents[fsmId])
				{
					if((desc.target - agentInfo.GetPos()).LengthSquared() >= desc.radiusSqr)
					{
						continue;
					}

					state = _machines[fsmId]->Advance(state, desc.eventToFire);

					if(state != oldState)
					{
						break;
					}
				}
			}

			if(state == oldState && _activeTimers.find(agentId) != _activeTimers.end())
			{
				for(auto & timer : _activeTimers[agentId])
				{
					timer.timeLeft -= timeStep;
					if(timer.timeLeft <= 0)
					{
						state = _machines[fsmId]->Advance(state, timer.eventToFire);
						if(state != oldState)
						{
							break;
						}
					}
				}
			}

			if(state != oldState)
			{
				_activeTimers[agentId].clear();
				if(_timerEvents.find(fsmId) != _timerEvents.end())
				{
					for(auto & desc : _timerEvents[fsmId])
					{
						if(state != desc.state)
						{
							continue;
						}

						std::uniform_real_distribution<float> gen(desc.minTime, desc.maxTime);
						_activeTimers[agentId].push_back({desc.eventToFire, gen(_random_engine)});
					}
				}

				auto & actions = _gotoActions[fsmId];
				if(actions.find(state) != actions.end())
				{
					_sim->SetAgentGoal(agentId, goalFactory.CreatePointGoal(actions[state]));
				}

				auto & anyActions = _gotoAnyActions[fsmId];
				if(anyActions.find(state) != anyActions.end())
				{
					auto & goals = anyActions[state];

					std::uniform_int_distribution<int> gen(0, goals.size() - 1);
					_sim->SetAgentGoal(agentId, goalFactory.CreatePointGoal(goals[gen(_random_engine)]));
				}
			}

			p.second.state = state;
		}
	}
}
