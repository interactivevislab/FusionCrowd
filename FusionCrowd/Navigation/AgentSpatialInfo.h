#pragma once

#include "Math/Geometry2D.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Export/IRecording.h"
#include "Export/Export.h"

#include <memory>

namespace FusionCrowd
{
	struct AgentSpatialInfo
	{
	public:
		using Type = size_t;
		static const Type AGENT = 1;
		static const Type GROUP = 2;
		static const Type COLLIDE_ALL = AGENT | GROUP;

		inline bool CanCollide(const AgentSpatialInfo & target) const
		{
			return (collisionsLevel & target.type) != 0;
		}

	public:
		size_t id;
		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 velNew;
		DirectX::SimpleMath::Vector2 orient = DirectX::SimpleMath::Vector2(1.f, 0.f);

		float radius    = 0.19f;
		float maxSpeed  = 2.f;
		float maxAccel  = 0.5f;
		float prefSpeed = 1.04f;
		float maxAngVel = 1.0f;

		bool inertiaEnabled = true;

		Type type = AGENT;
		Type collisionsLevel = COLLIDE_ALL;

		Agents::PrefVelocity prefVelocity = Agents::PrefVelocity(DirectX::SimpleMath::Vector2(1.f, 0.f), prefSpeed, DirectX::SimpleMath::Vector2(0.f, 0.f));

		std::unique_ptr<Math::Geometry2D> neighbourSearchShape;

		bool useNavMeshObstacles = true;

		AgentSpatialInfo() :
			neighbourSearchShape(std::make_unique<Math::DiskShape>(DirectX::SimpleMath::Vector2(0.f, 0.f), 4.0f))
		{ }

		AgentSpatialInfo(const AgentSpatialInfo & other) :
			id(other.id),
			pos(other.pos),
			vel(other.vel),
			velNew(other.velNew),
			orient(other.orient),
			radius(other.radius),
			maxSpeed(other.maxSpeed),
			prefSpeed(other.prefSpeed),
			maxAngVel(other.maxAngVel),
			inertiaEnabled(other.inertiaEnabled),
			type(other.type),
			collisionsLevel(other.collisionsLevel),
			prefVelocity(other.prefVelocity),
			neighbourSearchShape(std::unique_ptr<Math::Geometry2D>(other.neighbourSearchShape->Clone())),
			useNavMeshObstacles(other.useNavMeshObstacles)
		{ }

		AgentSpatialInfo & operator=(const AgentSpatialInfo & other)
		{
			id = other.id;
			pos = other.pos;
			vel = other.vel;
			velNew = other.velNew;
			orient = other.orient;
			radius = other.radius;
			maxSpeed = other.maxSpeed;
			prefSpeed = other.prefSpeed;
			maxAngVel = other.maxAngVel;
			inertiaEnabled = other.inertiaEnabled;
			type = other.type;
			collisionsLevel = other.collisionsLevel;
			prefVelocity = other.prefVelocity;
			neighbourSearchShape = std::unique_ptr<Math::Geometry2D>(other.neighbourSearchShape->Clone());
			useNavMeshObstacles = other.useNavMeshObstacles;

			return *this;
		}
	};
}