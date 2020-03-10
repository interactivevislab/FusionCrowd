#pragma once

#include "Math/Geometry2D.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Export/IRecording.h"
#include "Export/Export.h"

namespace FusionCrowd
{
	struct AgentSpatialInfo
	{
		enum CollisionLevel
		{
			AGENT, GROUP
		};

		size_t id;
		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 velNew;
		DirectX::SimpleMath::Vector2 orient = DirectX::SimpleMath::Vector2(1.f, 0.f);

		float radius    = 0.19f;
		float maxSpeed  = 2.f;
		float maxAccel  = 0.5f;
		float prefSpeed = 1.04f;
		float maxAngVel = 1.0f;

		bool inertiaEnabled = true;
		CollisionLevel collisionsLevel = AGENT;

		Agents::PrefVelocity prefVelocity = Agents::PrefVelocity(DirectX::SimpleMath::Vector2(1.f, 0.f), prefSpeed, DirectX::SimpleMath::Vector2(0.f, 0.f));

		Math::Geometry2D* neighbourSearchShape = new Math::DiskShape(DirectX::SimpleMath::Vector2(0.f, 0.f), 6.0f);

		bool useNavMeshObstacles = true;
	};
}