#pragma once

#include "TrafficLight.h"
#include "d3d11.h"
#include "SimpleMath.h"

class TrafficLightsBunch
{
public:
	TrafficLightsBunch();
	TrafficLightsBunch(float, float, float);
	TrafficLight* GetProperLight(DirectX::SimpleMath::Vector2);
	void UpdateAllLights(float);

private:

	enum dirs {north, east, south, west};

	const DirectX::SimpleMath::Vector2 directions[4] = { {0,1}, {1,0}, {0,-1}, {-1,0} };
	TrafficLight* lights[4]; //four traffic lights on crossroad

};