#pragma once

#include "../Config.h"

class FUSION_CROWD_API INavComponent
{
public:
	INavComponent() {};

	virtual void SetPrefVelocity() {};

	virtual ~INavComponent() {};
};
