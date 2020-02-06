#pragma once

#include "Math/Util.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Export/IRecording.h"
#include "Export/Export.h"

namespace FusionCrowd
{
	struct AgentSpatialInfo
	{
		size_t id;
		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 velNew;
		DirectX::SimpleMath::Vector2 orient = DirectX::SimpleMath::Vector2(1.f, 0.f);

		float radius    = 1.19f;
		float maxSpeed  = 2.f;
		float maxAccel  = 5.f;
		float prefSpeed = 1.04f;
		float maxAngVel = 6.28f;

		bool inertiaEnabled = true;

		Agents::PrefVelocity prefVelocity = Agents::PrefVelocity(DirectX::SimpleMath::Vector2(1.f, 0.f), prefSpeed, DirectX::SimpleMath::Vector2(0.f, 0.f));
	};
}