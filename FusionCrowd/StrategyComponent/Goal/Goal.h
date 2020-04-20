#pragma once

#include "Math/Geometry2D.h"
#include "Math/Util.h"
#include "TacticComponent/Path/PrefVelocity.h"

#include <memory>

namespace FusionCrowd
{
	class Goal
	{
	protected:
		Goal(size_t id, std::shared_ptr<Math::Geometry2D> geometry);

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
		std::shared_ptr<Math::Geometry2D> _geometry;
	};

	class GoalFactory
	{
	public:
		Goal CreatePointGoal(const DirectX::SimpleMath::Vector2 & p);
		Goal CreateDiscGoal(const DirectX::SimpleMath::Vector2 & center, float R);
		Goal CreateGeometryGoal(std::shared_ptr<Math::Geometry2D> geometry);
	private:
		size_t goalId = 0;
	};
}
