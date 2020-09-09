#include "TrafficLight.h"

TrafficLight::TrafficLight(Lights currentLight)
{
	gTime = 10.0f;
	yTime = 3.0f;
	rTime = 10.0f;
	prevLight = curLight = currentLight;
	timePassed = 0.0f;
}

TrafficLight::TrafficLight(float rT, float yT, float gT, Lights light)
{
	rTime = rT;
	yTime = yT;
	gTime = gT;
	prevLight = curLight = light;
	timePassed = 0.0f;
}

TrafficLight::Lights TrafficLight::GetCurLight()
{
	return curLight;
}

void TrafficLight::UpdateLights(float dt)
{
	timePassed += dt;

	switch (curLight)
	{
	case Lights::red:
		if (timePassed > rTime)
		{
			prevLight = curLight;
			timePassed = 0.0f;
			curLight = Lights::yellow;
		}
		return;
	case Lights::yellow:
		if (timePassed > yTime)
		{
			prevLight == Lights::red ? curLight = Lights::green : curLight = Lights::red;
			timePassed = 0.0f;
		}
		return;
	case Lights::green:
		if (timePassed > gTime)
		{
			prevLight = curLight;
			timePassed = 0.0f;
			curLight = Lights::yellow;
		}
		return;
	}
}

void TrafficLight::SetTimePassed(float time)
{
	timePassed = time;
}