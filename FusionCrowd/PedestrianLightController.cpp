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
		std::map<size_t, FusionCrowd::Agent>& agents, FusionCrowd::Simulator* simulator, 
		std::shared_ptr<NavSystem> navSystem,
		GoalFactory& goalFactory)  {
		if (lights.size() == 0) return;
		std::vector<size_t> to_unstop;
		for (auto& pair : _saved_goals) {
			if (agents.at(pair.first).currentGoal.getCentroid() == pair.second.getCentroid())
			{
				bool crossedNorth;
				if (_blocks_lights_bunch.at(pair.first)->PointInCross(navSystem->GetSpatialInfo(pair.first).GetPos(), _blocks_lights_position.at(pair.first), crossedNorth)) continue;
				to_unstop.push_back(pair.first);
				continue;
			}
			if (_blocks_lights.at(pair.first)->GetCurLight() == TrafficLight::Lights::green) {
				to_unstop.push_back(pair.first);
			}
		}
		for (auto id : to_unstop) {
			simulator->SetAgentGoal(id, std::move(_saved_goals.at(id)));
			_saved_goals.erase(id);
			_blocks_lights.erase(id);
			_blocks_lights_position.erase(id);
			_blocks_lights_bunch.erase(id);
		}

		for (auto& pair : agents) {
			if (_saved_goals.count(pair.first) > 0) continue;
			auto& agent = pair.second;
			if (agent.tacticComponent.lock()->GetId() != ComponentIds::NAVMESH_ID) continue;
			auto& spatialInfo = navSystem->GetSpatialInfo(agent.id);
			for (auto& light_pair : lights) {
				auto& light_bunch = light_pair.second;
				auto light_pos = navSystem->GetNavGraph()->GetNode(light_pair.first).position;
				if (light_pos.Distance(light_pos, spatialInfo.GetPos()) > LIGHT_SENSITIVE_RADIUS) continue;
				bool crossedNorth;
				DirectX::SimpleMath::Vector2 lightDir;
				if (!light_bunch->PointInCross(spatialInfo.GetPos(), light_pos, crossedNorth)) continue;
				//auto& light_bunch = light_pair.second;
				auto light = light_bunch->GetProperLight(spatialInfo.GetVel(), crossedNorth, lightDir);
				if (light->GetCurLight() != TrafficLight::Lights::green) {
					_saved_goals.insert({ agent.id, agent.currentGoal });
					_blocks_lights.insert({ agent.id, light });
					_blocks_lights_position.insert({ agent.id, light_pos });
					auto goal = goalFactory.CreatePointGoal(spatialInfo.GetPos() + lightDir*2);
					simulator->SetAgentGoal(agent.id, std::move(goal));
				}
				else
				{
					_saved_goals.insert({ agent.id, agent.currentGoal });
					_blocks_lights.insert({ agent.id, light });
					_blocks_lights_position.insert({ agent.id, light_pos });
					_blocks_lights_bunch.insert({ agent.id, light_bunch });
					//auto goal = goalFactory.CreatePointGoal(spatialInfo.GetPos());
					//simulator->SetAgentGoal(agent.id, std::move(goal));
				}

			}
		}
	}
}