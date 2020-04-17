#pragma once

#include "Math/Util.h"

#include "AgentSpatialInfo.h"

namespace FusionCrowd
{
	struct NeighborInfo
	{
		size_t id;

		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 orient;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 prefVel;

		float radius;

		bool inertiaEnabled;
		AgentSpatialInfo::Type collisionsLevel;

		NeighborInfo(const AgentSpatialInfo& agent);

		/*
		NeighborInfo(const NeighborInfo& other) = default;
		NeighborInfo& operator=(const NeighborInfo& other) = default;
		*/
	};
}