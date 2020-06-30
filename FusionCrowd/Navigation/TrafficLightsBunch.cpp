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