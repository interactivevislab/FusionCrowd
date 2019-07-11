#include "HelbingParameters.h"


namespace FusionCrowd
{
	namespace Helbing
	{
		// These values come directly from the Helbing 2000 paper
		float	HelbingParameters::AGENT_SCALE = 2000.f;
		float	HelbingParameters::OBST_SCALE = 2000.f;
		float	HelbingParameters::REACTION_TIME = 0.5f;
		float	HelbingParameters::BODY_FORCE = 1.2e5f;
		float	HelbingParameters::FRICTION = 2.4e5f;
		float	HelbingParameters::FORCE_DISTANCE = 0.08f;


		HelbingParameters::HelbingParameters()
		{
		}


		HelbingParameters::~HelbingParameters()
		{
		}
	}
}
