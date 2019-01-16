#pragma once

class INavComponent
{
public:
	INavComponent() {};

	virtual void SetPrefVelocity() {};

	virtual ~INavComponent() {};
};
