#pragma once

#include "Config.h"
#include "Math/Util.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Util/IRecording.h"
#include "Util/PublicSpatialInfo.h"

namespace FusionCrowd
{
	struct AgentSpatialInfo
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
		float maxAngVel = 6.28f;

		Agents::PrefVelocity prefVelocity = Agents::PrefVelocity(DirectX::SimpleMath::Vector2(1.f, 0.f), prefSpeed, DirectX::SimpleMath::Vector2(0.f, 0.f));

		AgentSpatialInfo() { }
		AgentSpatialInfo(const AgentSpatialInfo& other) = default;
		AgentSpatialInfo& operator=(AgentSpatialInfo& other) = default;

		~AgentSpatialInfo() = default;

		PublicSpatialInfo ToPublicInfo() const
		{
			PublicSpatialInfo publicInfo;

			publicInfo.id = id;

			publicInfo.posX = pos.x;
			publicInfo.posY = pos.y;

			publicInfo.velX = vel.x;
			publicInfo.velY = vel.y;

			publicInfo.orientX = orient.x;
			publicInfo.orientY = orient.y;
			publicInfo.radius = radius;

			return publicInfo;
		}
	};
}