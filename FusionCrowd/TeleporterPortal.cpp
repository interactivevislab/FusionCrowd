#include "TeleporterPortal.h"

namespace FusionCrowd
{
	TeleporterPortal::TeleporterPortal(const Goal& other, const Vector2& teleportTo)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(other.getGeometry()->Clone());
		_id = other.getID();
		_portalLocation = other.getCentroid();
		_teleportLocation = teleportTo;
	}

	TeleporterPortal::TeleporterPortal(const Vector2& location, const Math::Geometry2D* geometry, const Vector2& teleportTo)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(geometry->Clone());
		_portalLocation = location;
		_teleportLocation = teleportTo;
	}

	TeleporterPortal::TeleporterPortal(const Vector2& location, const Vector2& teleportTo)
	{
		_portalLocation = location;
		_teleportLocation = teleportTo;
	}
}