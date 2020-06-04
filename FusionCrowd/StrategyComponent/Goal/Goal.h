#pragma once

#include "Math/Shapes/Geometry2D.h"
#include "Math/Util.h"
#include "TacticComponent/PrefVelocity.h"

#include "Export/Math/Shapes.h"

#include <memory>

namespace FusionCrowd
{
	class Goal
	{
	public:
		Goal(const Goal & other);
		Goal& operator=(const Goal & other);

	protected:
		Goal(size_t id, std::unique_ptr<Math::Geometry2D> geometry);

		friend class GoalFactory;

	public:
		float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const;

		void setDirections(const DirectX::SimpleMath::Vector2 & q, float r, Agents::PrefVelocity & directions) const;

		DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const;
		DirectX::SimpleMath::Vector2 getCentroid() const;

		size_t getID() const;
		const Math::Geometry2D * getGeometry() const;

	protected:
		size_t _id;
		std::unique_ptr<Math::Geometry2D> _geometry;
	};

	class GoalFactory
	{
	public:
		Goal CreatePointGoal(const DirectX::SimpleMath::Vector2 & p);
		Goal CreatePointGoal(const Point & p);

		Goal CreateDiscGoal(const DirectX::SimpleMath::Vector2 & center, float R);
		Goal CreateDiscGoal(const Disk & d);

		Goal CreateRectGoal(const Rect & t);

		Goal CreateGeometryGoal(std::unique_ptr<Math::Geometry2D> geometry);

	private:
		friend class Simulator;
		GoalFactory();

	private:
		size_t goalId = 0;
	};
}
