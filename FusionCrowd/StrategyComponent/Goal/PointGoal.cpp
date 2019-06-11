#include "PointGoal.h"
#include "Math/Geometry2D.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	PointGoal::PointGoal() : Goal()
	{
	}

	PointGoal::PointGoal(const DirectX::SimpleMath::Vector2 & p) : Goal()
	{
		_geometry = new FusionCrowd::Math::PointShape(p);
	}

	PointGoal::PointGoal(float x, float y) : Goal()
	{
		_geometry = new FusionCrowd::Math::PointShape(DirectX::SimpleMath::Vector2(x, y));
	}
}