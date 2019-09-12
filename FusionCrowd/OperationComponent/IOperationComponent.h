#pragma once

#include "Config.h"
#include "ComponentId.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API IOperationComponent
	{
	public:
		virtual ComponentId GetId() = 0;
		virtual void AddAgent(size_t id) = 0;
		virtual bool DeleteAgent(size_t id) = 0;

		virtual void Update(float timeStep) = 0;
		virtual ~IOperationComponent() { };
	};
}