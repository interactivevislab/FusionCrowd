#pragma once

class TrafficLight
{
public:
	TrafficLight(int currentLight = red);
	TrafficLight(float, float, float, int currentLight = red);
	int GetCurLight();
	void UpdateLights(float);

	enum lights { red, yellow, green };

private:
	float rTime, yTime, gTime; //Time of lighting in seconds
	int curLight;
	float timePassed;
	int prevLight;
};