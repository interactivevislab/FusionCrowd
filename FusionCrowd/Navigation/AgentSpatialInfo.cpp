#include "AgentSpatialInfo.h"

#include "Math/Shapes/DiskShape.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	AgentSpatialInfo::AgentSpatialInfo() : neighbourSearchShape(std::make_unique<Math::DiskShape>(Vector2(0.f, 0.f), 4.0f))
	{ }

	AgentSpatialInfo::AgentSpatialInfo(const AgentSpatialInfo& other) :
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

	AgentSpatialInfo & AgentSpatialInfo::operator=(const AgentSpatialInfo & other)
	{
		id  = other.id;
		pos = other.pos;
		vel = other.vel;
		velNew = other.velNew;
		orient = other.orient;
		radius = other.radius;
		maxSpeed  = other.maxSpeed;
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

	AgentSpatialInfo::AgentSpatialInfo(AgentSpatialInfo && other):
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
		neighbourSearchShape(std::move(other.neighbourSearchShape)),
		useNavMeshObstacles(other.useNavMeshObstacles)
	{ }

	AgentSpatialInfo & AgentSpatialInfo::operator=(AgentSpatialInfo && other)
	{
		id  = other.id;
		pos = other.pos;
		vel = other.vel;
		velNew = other.velNew;
		orient = other.orient;
		radius = other.radius;
		maxSpeed  = other.maxSpeed;
		prefSpeed = other.prefSpeed;
		maxAngVel = other.maxAngVel;
		inertiaEnabled = other.inertiaEnabled;
		type = other.type;
		collisionsLevel = other.collisionsLevel;
		prefVelocity = other.prefVelocity;
		neighbourSearchShape = std::move(other.neighbourSearchShape);
		useNavMeshObstacles = other.useNavMeshObstacles;

		return *this;
	}

	void AgentSpatialInfo::SetPos(Vector2 pos)
	{
		this->pos = pos;
	}

	void AgentSpatialInfo::setOverlaping(bool newVal)
	{
		_isOverlaping = newVal;
	}

	void AgentSpatialInfo::Update(Vector2 pos, Vector2 vel, Vector2 orient)
	{
		this->pos    = pos;
		this->vel    = vel;
		this->orient = orient;
	}
}