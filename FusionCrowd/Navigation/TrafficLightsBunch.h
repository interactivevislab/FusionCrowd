#pragma once

#include "Export/Math/Shapes.h"
#include "Math/Shapes/Geometry2D.h"
#include <memory>

#include "TrafficLight.h"
#include "d3d11.h"
#include "SimpleMath.h"

class TrafficLightsBunch
{
public:
	TrafficLightsBunch();
	TrafficLightsBunch(float redTime, float yellowTime, float greenTime);
	TrafficLight* GetProperLight(DirectX::SimpleMath::Vector2 vehicleOrientation);
	TrafficLight* GetProperLight(DirectX::SimpleMath::Vector2 vehicleOrientation, bool crossedNorth, DirectX::SimpleMath::Vector2& lightDir);
	void UpdateAllLights(float deltaTime);
	void DeleteLights();

	void ConstructPedestrianCheckZones(float northDirX, float northDirY, float westDirX, float westDirY, float northWidth, float westWidth);

	bool PointInCross(DirectX::SimpleMath::Vector2 P, DirectX::SimpleMath::Vector2 shift, bool& crossedNorth);

	DirectX::SimpleMath::Vector2 northLeftForward;
	DirectX::SimpleMath::Vector2 northLeftBackward;
	DirectX::SimpleMath::Vector2 northRightForward;
	DirectX::SimpleMath::Vector2 northRightBackward;

	DirectX::SimpleMath::Vector2 westLeftForward;
	DirectX::SimpleMath::Vector2 westLeftBackward;
	DirectX::SimpleMath::Vector2 westRightForward;
	DirectX::SimpleMath::Vector2 westRightBackward;

private:

	enum class Directions {north, east, south, west};

	const DirectX::SimpleMath::Vector2 directions[4] = { {1,-1}, {1,1}, {-1,1}, {-1,-1} };
	TrafficLight* lights[4]; //four traffic lights on crossroad

};