#include "TrafficLightsBunch.h"

TrafficLightsBunch::TrafficLightsBunch()
{
	for (int i = 0; i < 4; i++)
	{

		if (i == north || i == south)
		{
			lights[i] = new TrafficLight;
		}
		else
		{
			lights[i] = new TrafficLight(TrafficLight::green);
		}
	}
}

TrafficLightsBunch::TrafficLightsBunch(float rT, float yT, float gT)
{
	for (int i = 0; i < 4; i++)
	{
		if (i == north || i == south)
		{
			lights[i] = new TrafficLight(rT, yT, gT);
		}
		else
		{
			lights[i] = new TrafficLight(gT, yT, rT);
		}
	}
}

TrafficLight* TrafficLightsBunch::GetProperLight(DirectX::SimpleMath::Vector2 actorDirection)
{
	float dotProduct;
	float minDot = INFINITY;
	TrafficLight* res = new TrafficLight;

	for (int i = 0; i < 4; i++)
	{
		dotProduct = actorDirection.Dot(directions[i]);
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