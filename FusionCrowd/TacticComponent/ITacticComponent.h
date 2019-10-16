#pragma once

#include "Export/Config.h"
#include "Export/ComponentId.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API ITacticComponent
	{
	public:
		virtual ComponentId GetId() = 0;

		virtual void AddAgent(size_t id) = 0;
		virtual bool DeleteAgent(size_t id) = 0;

		virtual void Update(float timeStep) = 0;
		virtual ~ITacticComponent() { };
	};
}
