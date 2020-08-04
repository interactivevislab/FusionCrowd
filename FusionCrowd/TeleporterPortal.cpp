#include "TeleporterPortal.h"

namespace FusionCrowd
{
	TeleporterPortal::TeleporterPortal(const Goal& other, const Vector2& teleportTo, size_t backwayId)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(other.getGeometry()->Clone());
		_id = other.getID();
		_portalLocation = other.getCentroid();
		_teleportLocation = teleportTo;
		_backwayTeleporterIndex = backwayId;
	}

	TeleporterPortal::TeleporterPortal(const Vector2& location, const Math::Geometry2D* geometry, const Vector2& teleportTo)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(geometry->Clone());
		_portalLocation = location;
		_teleportLocation = teleportTo;
	}

	TeleporterPortal::TeleporterPortal(const Vector2& location, const Vector2& teleportTo, size_t backwayId, size_t toRoomId)
	{
		_portalLocation = location;
		_teleportLocation = teleportTo;
		_backwayTeleporterIndex = backwayId;
		_leadsToRoomWithId = toRoomId;
	}
}