#pragma once
#include <math.h>

#include "Export/Config.h"

namespace FusionCrowd
{
	/*!
	*	@brief		A convenient definition of infinity.
	*/
	extern FUSION_CROWD_API const float INFTY;

	/*!
		*	@brief		pi.
		*/
	extern FUSION_CROWD_API const float PI;

	/*!
		*	@brief		2 * pi.
		*/
	extern FUSION_CROWD_API const float TWOPI;

	/*!
		*	@brief		pi / 2.
		*/
	extern FUSION_CROWD_API const float HALFPI;
	/*!
		*	@brief		Scale factor for converting degrees to radians.
		*/
	extern FUSION_CROWD_API const float DEG_TO_RAD;

	/*!
		*	@brief		Scale factor for converting radians to degrees.
		*/
	extern FUSION_CROWD_API const float RAD_TO_DEG;

	/*!
		*	@brief		Suitably small number for testing for functional zero values.
		*/
	FUSION_CROWD_API const float EPS = 0.00001f;
}
