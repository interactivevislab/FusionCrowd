#include "PedestrianLightController.h"
//#include "TrafficLightsBunch.h"
#include "Simulator.h"

namespace FusionCrowd {
	PedestrianLightController::PedestrianLightController()
	{
	}


	PedestrianLightController::~PedestrianLightController()
	{
	}

	void PedestrianLightController::Update(std::map<size_t, TrafficLightsBunch*>&  const lights,
		std::map<size_t, FusionCrowd::Agent> agents, FusionCrowd::Simulator* simulator, 
		std::shared_ptr<NavSystem> navSystem,
		GoalFactory& goalFactory)  {
		if (lights.size() == 0) return;
		std::vector<size_t> to_unstop;
		for (auto& pair : _saved_goals) {
			if (_blocks_lights.at(pair.first)->GetCurLight() == TrafficLight::Lights::green) {
				to_unstop.push_back(pair.first);
			}
		}
		for (auto id : to_unstop) {
			simulator->SetAgentGoal(id, std::move(_saved_goals.at(id)));
			_saved_goals.erase(id);
			_blocks_lights.erase(id);
		}

		for (auto& pair : agents) {
			if (_saved_goals.count(pair.first) > 0) continue;
			auto& agent = pair.second;
			auto& spatialInfo = navSystem->GetSpatialInfo(agent.id);
			for (auto& light_pair : lights) {
				auto light_pos = navSystem->GetNavGraph()->GetNode(light_pair.first).position;
				if (light_pos.Distance(light_pos, spatialInfo.GetPos()) > LIGHT_SENSITIVE_RADIUS) continue;
				auto& light_bunch = light_pair.second;
				auto light = light_bunch->GetProperLight(spatialInfo.GetVel());
				if (light->GetCurLight() != TrafficLight::Lights::green) {
					_saved_goals.insert({ agent.id, agent.currentGoal });
					_blocks_lights.insert({ agent.id, light });
					auto goal = goalFactory.CreatePointGoal(spatialInfo.GetPos());
					simulator->SetAgentGoal(agent.id, std::move(goal));
				}

			}
		}
	}
}