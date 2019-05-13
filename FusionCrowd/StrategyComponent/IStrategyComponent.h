#pragma once

#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API IStrategyComponent
	{
	public:
		virtual void Update(float timeStep)
		{
		};

		virtual ~IStrategyComponent()
		{
		};
	};
}
