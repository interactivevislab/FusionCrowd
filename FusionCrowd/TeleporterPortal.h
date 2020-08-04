#pragma once
#include "StrategyComponent/Goal/Goal.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class TeleporterPortal
	{
	public:
		TeleporterPortal(const Goal& other, const Vector2& teleportTo, size_t backwayId);
		TeleporterPortal(const Vector2& location, const Math::Geometry2D* geometry, const Vector2& teleportTo);
		TeleporterPortal(const Vector2& location, const Vector2& teleportTo, size_t backwayId, size_t toRoomId);
		Vector2 _portalLocation;
		Vector2 _teleportLocation;
		size_t _backwayTeleporterIndex = -1;
		size_t _leadsToRoomWithId = -1;

	protected:
		size_t _id;
		std::unique_ptr<Math::Geometry2D> _geometry;

		
		Vector2 _agentPrimaryGoal;

	private:
		size_t goalId = 0;
	};
}

