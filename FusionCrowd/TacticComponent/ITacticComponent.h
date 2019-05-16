#pragma once

#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API ITacticComponent
	{
	public:
		virtual void AddAgent(size_t id) = 0;
		virtual bool RemoveAgent(size_t id) = 0;

		virtual void Update(float timeStep) = 0;
		virtual ~ITacticComponent() { };
	};
}
