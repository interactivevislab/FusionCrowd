#pragma once

#include "Math/Geometry2D.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	// Forward declaration
	class GoalSet;

	class FUSION_CROWD_API Goal
	{
	public:
		Goal() : _weight(1.f), _capacity(MAX_CAPACITY), _id(-1),
			_goalSet(0x0), _population(0), _geometry(0x0) {}
		~Goal();

		void destroy() { delete this; }

		static const size_t MAX_CAPACITY = -1;
		friend class GoalSet;

	public:
		virtual std::string getStringId() const = 0;

		float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const
		{
			return _geometry->squaredDistance(pt);
		}
		void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
			Agents::PrefVelocity & directions) const
		{
			return _geometry->setDirections(q, r, directions);
		}

		DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const
		{
			return _geometry->getTargetPoint(q, r);
		}

		DirectX::SimpleMath::Vector2 getCentroid() const { return _geometry->getCentroid(); }

		bool hasCapacity() const;
		void assign(const Agent * agent);
		void free();
		void setGeometry(Math::Geometry2D * geometry);
		inline void setGoalSet(GoalSet * goalSet) { _goalSet = goalSet; }
		inline GoalSet * getGoalSet() { return _goalSet; }
		inline const GoalSet * getGoalSet() const { return _goalSet; }
		inline void setWeight(float weight) { _weight = weight; }
		inline float getWeight() const { return _weight; }
		inline void setCapacity(size_t capacity) { _capacity = capacity; }
		inline size_t getCapacity() const { return _capacity; }
		inline void setID(size_t id) { _id = id; }
		inline size_t getID() const { return _id; }
		inline const Math::Geometry2D * getGeometry() const { return _geometry; }

		friend class GoalSet;
	protected:
		float	_weight;
		size_t	_capacity;
		size_t	_id;
		GoalSet * _goalSet;
		mutable size_t	_population;
		Math::Geometry2D * _geometry;
	};
}
