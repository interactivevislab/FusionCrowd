#pragma once

#include "Config.h"
#include "Math/Util.h"
#include "TacticComponent/Path/PrefVelocity.h"

namespace FusionCrowd
{
	struct FUSION_CROWD_API AgentSpatialInfo
	{
		size_t id;
		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 velNew;
		DirectX::SimpleMath::Vector2 orient = DirectX::SimpleMath::Vector2(1.f, 0.f);

		float radius    = 0.19f;
		float maxSpeed  = 2.5f;
		float maxAccel  = 2.f;
		float prefSpeed = 1.34f;
		float maxAngVel = 360.f;

		Agents::PrefVelocity prefVelocity = Agents::PrefVelocity(DirectX::SimpleMath::Vector2(1.f, 0.f), prefSpeed, DirectX::SimpleMath::Vector2(0.f, 0.f));
	};
}