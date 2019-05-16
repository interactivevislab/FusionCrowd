#pragma once

#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API IOperationComponent
	{
	public:
		virtual void Update(float timeStep) {}
		virtual ~IOperationComponent() {}
	};
}