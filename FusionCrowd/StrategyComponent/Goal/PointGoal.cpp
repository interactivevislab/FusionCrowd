#include "PointGoal.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Math/Geometry2D.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	//const std::string PointGoal::NAME = "point";

	PointGoal::PointGoal() : Goal()
	{
	}

	PointGoal::PointGoal(const Vector2 & p) : Goal()
	{
		_geometry = new FusionCrowd::Math::PointShape(p);
	}

	PointGoal::PointGoal(float x, float y) : Goal()
	{
		_geometry = new FusionCrowd::Math::PointShape(Vector2(x, y));
	}
}