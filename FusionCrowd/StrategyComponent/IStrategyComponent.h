#pragma once

#include "Config.h"
#include <string>

namespace FusionCrowd
{
	class FUSION_CROWD_API IStrategyComponent
	{
	public:
		virtual void AddAgent(size_t id) = 0;
		virtual bool RemoveAgent(size_t id) = 0;
		virtual void Update(float timeStep) = 0;
		virtual std::string GetName() = 0;

		virtual ~IStrategyComponent()
		{
		};
	};
}
