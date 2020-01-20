#pragma once

#include "Export/Config.h"
#include "Export/ComponentId.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API ITacticComponent
	{
	public:
		virtual ComponentId GetId() = 0;

		virtual void AddAgent(size_t id) = 0;
		virtual bool DeleteAgent(size_t id) = 0;

		virtual void Update(float timeStep) = 0;
		virtual DirectX::SimpleMath::Vector2 GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p) = 0;
		virtual ~ITacticComponent() { };
	};
}
