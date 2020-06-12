#pragma once

#include "TrafficLight.h"
#include "d3d11.h"
#include "SimpleMath.h"

class TrafficLightsBunch
{
public:
	TrafficLightsBunch();
	TrafficLightsBunch(float redTime, float yellowTime, float greenTime);
	TrafficLight* GetProperLight(DirectX::SimpleMath::Vector2 vehicleOrientation);
	void UpdateAllLights(float deltaTime);
	void DeleteLights();

private:

	enum class Directions {north, east, south, west};

	const DirectX::SimpleMath::Vector2 directions[4] = { {0,1}, {1,0}, {0,-1}, {-1,0} };
	TrafficLight* lights[4]; //four traffic lights on crossroad

};