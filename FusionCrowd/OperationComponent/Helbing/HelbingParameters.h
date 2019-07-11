#pragma once
#include "Config.h"

namespace FusionCrowd
{
	namespace Helbing
	{
		class FUSION_CROWD_API HelbingParameters
		{
		public:
			HelbingParameters();
			~HelbingParameters();

			static float AGENT_SCALE;
			static float OBST_SCALE;
			static float REACTION_TIME;
			static float BODY_FORCE;
			static float FRICTION;
			static float FORCE_DISTANCE;
		};
	}
}

