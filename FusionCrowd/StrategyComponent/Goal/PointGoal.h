#pragma once

#include "Goal.h"
#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API PointGoal : public Goal
	{
	public:
		PointGoal();
		PointGoal(const DirectX::SimpleMath::Vector2 & p);
		PointGoal(float x, float y);
		virtual std::string getStringId() const { return "point"; }

		//static const std::string NAME;
	};
}

