#pragma once

#include "Math/Util.h"
#include "Navigation/AgentSpatialInfo.h"


namespace FusionCrowd
{
	class IGroupShape
	{
	public:
		virtual void AddAgent(size_t agentId, AgentSpatialInfo& info) = 0;
		virtual void RemoveAgent(size_t agentId) = 0;
		virtual DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const = 0;
		virtual size_t GetSize() const = 0;
		virtual float GetRadius() const = 0;
	};
}
