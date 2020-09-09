#include "TrafficLightsBunch.h"

TrafficLightsBunch::TrafficLightsBunch()
{
	for (int i = 0; i < 4; i++)
	{
		if ((Directions) i == Directions::north || (Directions) i == Directions::south)
		{
			lights[i] = new TrafficLight(TrafficLight::Lights::red);
		}
		else
		{
			lights[i] = new TrafficLight(TrafficLight::Lights::green);
			//lights[i] = new TrafficLight(TrafficLight::Lights::red);
			//lights[i]->SetTimePassed(35.0f);
		}
	}
}

TrafficLightsBunch::TrafficLightsBunch(float rT, float yT, float gT)
{
	for (int i = 0; i < 4; i++)
	{
		if ((Directions)i == Directions::north || (Directions)i == Directions::south)
		{
			lights[i] = new TrafficLight(rT, yT, gT);
		}
		else
		{
			lights[i] = new TrafficLight(gT, yT, rT);
		}
	}
}

TrafficLight* TrafficLightsBunch::GetProperLight(DirectX::SimpleMath::Vector2 vehicleDirection)
{
	float dotProduct;
	float minDot = INFINITY;
	TrafficLight* res = NULL;

	for (int i = 0; i < 4; i++)
	{
		dotProduct = vehicleDirection.Dot(directions[i]);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			res = lights[i];
		}
	}

	return res;
}

TrafficLight* TrafficLightsBunch::GetProperLight(DirectX::SimpleMath::Vector2 vehicleDirection, bool crossedNorth, DirectX::SimpleMath::Vector2 & lightDir)
{
	float dotProduct;
	float minDot = INFINITY;
	TrafficLight* res = NULL;

	for (int i = 0; i < 4; i++)
	{
		if (crossedNorth)
		{
			if ((Directions)i != Directions::east && (Directions)i != Directions::west) continue;
		}
		else
		{
			if ((Directions)i != Directions::north && (Directions)i != Directions::south) continue;
		}
		dotProduct = vehicleDirection.Dot(directions[i]);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			res = lights[i];
			lightDir = directions[i];
		}
	}

	return res;
}

void TrafficLightsBunch::UpdateAllLights(float dt)
{
	for (auto light : lights)
	{
		light->UpdateLights(dt);
	}
}

void TrafficLightsBunch::DeleteLights()
{
	for (auto light : lights)
	{
		delete light;
	}
}

void TrafficLightsBunch::ConstructPedestrianCheckZones(float northDirX, float northDirY, float westDirX, float westDirY, float northWidth, float westWidth)
{
	auto northDir = DirectX::SimpleMath::Vector2(northDirX, northDirY);
	auto westDir = DirectX::SimpleMath::Vector2(westDirX, westDirY);
	auto normalNorth = DirectX::SimpleMath::Vector2(northDirY, -northDirX);
	auto normalWest = DirectX::SimpleMath::Vector2(westDirY, -westDirX);

	/*DirectX::SimpleMath::Vector2*/ northLeftForward = (normalNorth * northWidth * 0.5f) + (northDir * 15.0f);
	/*DirectX::SimpleMath::Vector2*/ northLeftBackward = (normalNorth * northWidth * 0.5f) + (northDir * -15.0f);
	/*DirectX::SimpleMath::Vector2*/ northRightForward = (normalNorth * -northWidth * 0.5f) + (northDir * 15.0f);
	/*DirectX::SimpleMath::Vector2*/ northRightBackward = (normalNorth * -northWidth * 0.5f) + (northDir * -15.0f);
								  
	/*DirectX::SimpleMath::Vector2*/ westLeftForward = (normalWest * westWidth * 0.5f) + (westDir * 15.0f);
	/*DirectX::SimpleMath::Vector2*/ westLeftBackward = (normalWest * westWidth * 0.5f) + (westDir * -15.0f);
	/*DirectX::SimpleMath::Vector2*/ westRightForward = (normalWest * -westWidth * 0.5f) + (westDir * 15.0f);
	/*DirectX::SimpleMath::Vector2*/ westRightBackward = (normalWest * -westWidth * 0.5f) + (westDir * -15.0f);
}

float isLeft(DirectX::SimpleMath::Vector2 P0, DirectX::SimpleMath::Vector2 P1, DirectX::SimpleMath::Vector2 P2)
{
	return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
}
bool TrafficLightsBunch::PointInCross(DirectX::SimpleMath::Vector2 P, DirectX::SimpleMath::Vector2 shift, bool & crossedNorth)
{
	 bool isInNorthRect = (isLeft(northRightForward + shift, northRightBackward + shift, P) > 0 && isLeft(northRightBackward + shift, northLeftBackward + shift, P) > 0 &&
		 isLeft(northLeftBackward + shift, northLeftForward + shift, P) > 0 && isLeft(northLeftForward + shift, northRightForward + shift, P) > 0);
	 bool isInWestRect = (isLeft(westRightForward + shift, westRightBackward + shift, P) > 0 && isLeft(westRightBackward + shift, westLeftBackward + shift, P) > 0 &&
		 isLeft(westLeftBackward + shift, westLeftForward + shift, P) > 0 && isLeft(westLeftForward + shift, westRightForward + shift, P) > 0);

	 crossedNorth = isInNorthRect;

	 return isInNorthRect || isInWestRect;
}