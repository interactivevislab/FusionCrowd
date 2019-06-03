#pragma once

#include "Config.h"
#include <string>

namespace FusionCrowd
{
	class FUSION_CROWD_API IOperationComponent
	{
	public:
		virtual std::string GetName() = 0;
		virtual void AddAgent(size_t id) = 0;
		virtual bool DeleteAgent(size_t id) = 0;

		virtual void Update(float timeStep) = 0;
		virtual ~IOperationComponent() { };
	};
}