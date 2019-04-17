#pragma once

#include "Config.h"

class FUSION_CROWD_API ITacticComponent
{
public:
	ITacticComponent() {};

	virtual void SetPrefVelocity() {};

	virtual ~ITacticComponent() {};
};
