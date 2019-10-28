#pragma once

#include "Export/Config.h"
#include "Export/ComponentId.h"
#include "Export/ModelAgentParams.h"


namespace FusionCrowd
{
	class FUSION_CROWD_API IStrategyComponent
	{
	public:
		virtual void AddAgent(size_t id) = 0;
		virtual bool RemoveAgent(size_t id) = 0;
		virtual void Update(float timeStep) = 0;

		virtual void SetAgentParams(size_t id, ModelAgentParams & params) = 0;

		virtual ComponentId GetId() = 0;

		virtual ~IStrategyComponent(){};
	};
}
