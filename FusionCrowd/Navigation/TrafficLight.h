#pragma once

class TrafficLight
{
public:
	enum class Lights { red, yellow, green };
public:
	TrafficLight(Lights currentLight = Lights::red);
	TrafficLight(float redTime, float yellowTime, float greenTime, Lights currentLight = Lights::red);
	Lights GetCurLight();
	void UpdateLights(float deltaTime);

	

private:
	float rTime, yTime, gTime; //Time of lighting in seconds
	Lights curLight, prevLight;
	float timePassed;
};