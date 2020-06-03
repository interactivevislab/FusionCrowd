#include "TrafficLight.h"

TrafficLight::TrafficLight(int currentLight)
{
	rTime = gTime = 10.0f;
	yTime = 3.0f;
	prevLight = curLight = currentLight;
	timePassed = 0.0f;
}

TrafficLight::TrafficLight(float rT, float yT, float gT, int light)
{
	rTime = rT;
	yTime = yT;
	gTime = gT;
	prevLight = curLight = light;
	timePassed = 0.0f;
}

int TrafficLight::GetCurLight()
{
	return curLight;
}

void TrafficLight::UpdateLights(float dt)
{
	timePassed += dt;

	switch (curLight)
	{
	case red:
		if (timePassed > rTime)
		{
			prevLight = curLight;
			timePassed = 0.0f;
			curLight = yellow;
		}
		return;
	case yellow:
		if (timePassed > yTime)
		{
			prevLight == red ? curLight = green : curLight = red;
			timePassed = 0.0f;
		}
		return;
	case green:
		if (timePassed > gTime)
		{
			prevLight = curLight;
			timePassed = 0.0f;
			curLight = yellow;
		}
		return;
	}

}