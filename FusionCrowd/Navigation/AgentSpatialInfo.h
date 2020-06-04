#pragma once

#include "Math/Shapes/Geometry2D.h"
#include "TacticComponent/PrefVelocity.h"
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
		DirectX::SimpleMath::Vector2 velNew;

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

	public:
		AgentSpatialInfo();

		AgentSpatialInfo(const AgentSpatialInfo & other);
		AgentSpatialInfo & operator=(const AgentSpatialInfo & other);

		AgentSpatialInfo(AgentSpatialInfo && other);
		AgentSpatialInfo & operator=(AgentSpatialInfo && other);

		inline DirectX::SimpleMath::Vector2 GetPos()    const { return pos; }
		inline DirectX::SimpleMath::Vector2 GetVel()    const { return vel; }
		inline DirectX::SimpleMath::Vector2 GetOrient() const { return orient; }

		void SetPos(DirectX::SimpleMath::Vector2 pos);

		void Update(
			DirectX::SimpleMath::Vector2 pos,
			DirectX::SimpleMath::Vector2 vel,
			DirectX::SimpleMath::Vector2 orient
		);

		inline bool isOverlaping() const { return _isOverlaping; };
		void setOverlaping(bool newVal);

	private:
		bool _isOverlaping = false;
		DirectX::SimpleMath::Vector2 pos;
		DirectX::SimpleMath::Vector2 vel;
		DirectX::SimpleMath::Vector2 orient = DirectX::SimpleMath::Vector2(1.f, 0.f);
	};
}