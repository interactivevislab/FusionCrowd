#include "Goal.h"

#include "TacticComponent/PrefVelocity.h"

#include "Math/Shapes/RectShape.h"
#include "Math/Shapes/PointShape.h"
#include "Math/Shapes/DiskShape.h"

namespace FusionCrowd
{
	Goal::Goal(const Goal& other)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(other.getGeometry()->Clone());
		_id = other.getID();
	}

	Goal& Goal::operator=(const Goal& other)
	{
		_geometry = std::unique_ptr<Math::Geometry2D>(other.getGeometry()->Clone());
		_id = other.getID();

		return *this;
	}

	Goal::Goal(size_t id, std::unique_ptr<Math::Geometry2D> geometry) : _id(id), _geometry(std::move(geometry)) { }

	float Goal::squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const
	{
		return _geometry->squaredDistance(pt);
	}

	void Goal::setDirections(const DirectX::SimpleMath::Vector2 & q, float r, Agents::PrefVelocity & directions) const
	{
		return _geometry->setDirections(q, r, directions);
	}

	DirectX::SimpleMath::Vector2 Goal::getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const
	{
		return _geometry->getTargetPoint(q, r);
	}

	DirectX::SimpleMath::Vector2 Goal::getCentroid() const { return _geometry->getCentroid(); }

	const Math::Geometry2D* Goal::getGeometry() const
	{
		return _geometry.get();
	}

	size_t Goal::getID() const
	{
		return _id;
	}

	GoalFactory::GoalFactory() { }

	Goal GoalFactory::CreatePointGoal(const DirectX::SimpleMath::Vector2& p)
	{
		auto geometry = std::make_unique<Math::PointShape>(p);

		return Goal(goalId++, std::move(geometry));
	}

	Goal GoalFactory::CreatePointGoal(const Point& p)
	{
		auto geometry = std::make_unique<Math::PointShape>(p);

		return Goal(goalId++, std::move(geometry));
	}

	Goal GoalFactory::CreateDiscGoal(const Disk& d)
	{
		auto geometry = std::make_unique<Math::DiskShape>(d);

		return Goal(goalId++, std::move(geometry));
	}

	Goal GoalFactory::CreateDiscGoal(const DirectX::SimpleMath::Vector2& p, float R)
	{
		auto geometry = std::make_unique <Math::DiskShape>(p, R);

		return Goal(goalId++, std::move(geometry));
	}

	Goal GoalFactory::CreateRectGoal(const Rect& r)
	{
		auto geometry = std::make_unique <Math::RectShape>(r);

		return Goal(goalId++, std::move(geometry));
	}

	Goal GoalFactory::CreateGeometryGoal(std::unique_ptr<Math::Geometry2D> geometry)
	{
		return Goal(goalId++, std::move(geometry));
	}
}
