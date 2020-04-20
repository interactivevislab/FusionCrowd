#pragma once

#include <list>
#include "Math/Util.h"
#include "Math/consts.h"

namespace FusionCrowd
{
	// FORWARD DECLARATION
	class PortalPath;
	class FunnelPlanner;

	class FunnelApex
	{
	public:
		FunnelApex(size_t id, const DirectX::SimpleMath::Vector2& point) : _id(id), _pos(point)
		{
		}

		inline void set(size_t id, const DirectX::SimpleMath::Vector2& point)
		{
			_id = id;
			_pos = point;
		}

		friend class FunnelPlanner;
	protected:
		size_t _id;
		DirectX::SimpleMath::Vector2 _pos;
	};

	class FunnelEdge
	{
	public:
#ifdef SIMPLE_FUNNEL
		FunnelEdge(size_t id, const Vector2 & dir) :_id(id), _dir(dir) {}
#else
		FunnelEdge(size_t id, size_t end, const DirectX::SimpleMath::Vector2& dir,
		           const DirectX::SimpleMath::Vector2& origin) :
			_id(id), _endID(end), _dir(dir), _origin(origin)
		{
		}
#endif
		inline bool isOnLeft(const DirectX::SimpleMath::Vector2& dir) const
		{
			return Math::det(_dir, dir) > Math::EPS;
		}

		inline bool isOnRight(const DirectX::SimpleMath::Vector2& dir) const
		{
			return Math::det(dir, _dir) > Math::EPS;
		}

		inline void set(size_t id, const DirectX::SimpleMath::Vector2& dir)
		{
			_id = id;
			_dir = dir;
		}

		friend class FunnelPlanner;
	protected:
#ifdef SIMPLE_FUNNEL
	size_t	_id;
#else
		size_t _id;
		size_t _endID;
		DirectX::SimpleMath::Vector2 _origin;
#endif
		DirectX::SimpleMath::Vector2 _dir;
	};

	class FunnelPlanner
	{
	public:
		FunnelPlanner();
		~FunnelPlanner();
		void computeCrossing(float radius, const DirectX::SimpleMath::Vector2& startPos, PortalPath* path,
		                     size_t startPortal = 0);
#ifndef SIMPLE_FUNNEL
	protected:
		std::list<FunnelEdge> _left;
		std::list<FunnelEdge> _right;
#endif
	};
}
