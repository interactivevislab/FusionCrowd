#pragma once
#include <map>

#include "Simulator.h"
#include "Navigation/TrafficLightsBunch.h"
#include "Navigation/NavSystem.h"

namespace FusionCrowd
{

	class PedestrianLightController
	{
	public:
		PedestrianLightController();
		~PedestrianLightController();

		void Update(std::map<size_t, TrafficLightsBunch*>&  lights,
			std::map<size_t, Agent>& agents,
			Simulator* simulator,
			std::shared_ptr<NavSystem> navSystem,
			GoalFactory& goalFactory);
	private:
		float const LIGHT_SENSITIVE_RADIUS = 15.0f;
		std::map<size_t, Goal> _saved_goals;
		std::map<size_t, TrafficLight*> _blocks_lights;
		std::map<size_t, DirectX::SimpleMath::Vector2> _blocks_lights_position;
		std::map<size_t, TrafficLightsBunch*> _blocks_lights_bunch;
	};
}
