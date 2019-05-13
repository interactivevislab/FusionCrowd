#pragma once

#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API ITacticComponent
	{
	public:
		ITacticComponent()
		{
		};

		virtual void Update(float timeStep)
		{
		};

		virtual ~ITacticComponent()
		{
		};
	};
}
