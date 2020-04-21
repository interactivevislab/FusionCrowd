#pragma once

#include <istream>

#include "TacticComponent/PrefVelocity.h"
#include "NavMeshNode.h"
#include "Math/Util.h"

// Forward declarations
namespace FusionCrowd
{
	class NavMesh;
	class NavMeshNode;

	class NavMeshEdge
	{
	public:
		NavMeshEdge();
		~NavMeshEdge();

		inline DirectX::SimpleMath::Vector2 getP0() const { return _point; }
		inline DirectX::SimpleMath::Vector2 getP0(float dist) const { return _point + _dir * dist; }
		inline DirectX::SimpleMath::Vector2 getP1() const { return _point + _dir * _width; }

		inline DirectX::SimpleMath::Vector2 getP1(float dist) const
		{
			return _point + _dir * (_width - dist);
		}

		inline DirectX::SimpleMath::Vector2 getDirection() const { return _dir; }
		NavMeshNode* getFirstNode() const { return _node0; }
		NavMeshNode* getSecondNode() const { return _node1; }
		NavMeshNode* getOtherByID(unsigned int id) const;
		NavMeshNode* getOtherByPtr(const NavMeshNode* node);
		const NavMeshNode* getOtherByPtr(const NavMeshNode* node) const;

		inline void setPoint(const DirectX::SimpleMath::Vector2& p)
		{
			_point.x = p.x;
			_point.y = p.y;
		}

		inline void setDirection(const DirectX::SimpleMath::Vector2& d)
		{
			_dir.x = d.x;
			_dir.y = d.y;
		}

		inline void setWidth(float w) { _width = w; }
		inline float getWidth() const { return _width; }

		inline void setNodes(NavMeshNode* n0, NavMeshNode* n1)
		{
			_node0 = n0;
			_node1 = n1;
		}

		inline DirectX::SimpleMath::Vector2 getPoint(float t) const { return _point + t * _dir; }
		bool pointClear(const DirectX::SimpleMath::Vector2& pos, float radius, float param) const;
		DirectX::SimpleMath::Vector2 targetPoint(const DirectX::SimpleMath::Vector2& pos, float radius) const;
		DirectX::SimpleMath::Vector2 getClearDirection(const DirectX::SimpleMath::Vector2& pos, float radius,
		                                               const DirectX::SimpleMath::Vector2& dir) const;
		void setClearDirections(const DirectX::SimpleMath::Vector2& pos, float radius,
		                        const DirectX::SimpleMath::Vector2& dir, Agents::PrefVelocity& pVel) const;
		float getSqDist(const DirectX::SimpleMath::Vector2& pt) const;
		float getSqDist(const DirectX::SimpleMath::Vector2& pt, DirectX::SimpleMath::Vector2& nearPt) const;
		float getDist(const DirectX::SimpleMath::Vector2& pt) const { return sqrtf(getSqDist(pt)); }
		float getNodeDistance(float minWidth);
		inline float getNodeDistance() const { return _distance; }
		bool loadFromAscii(std::istream& f, DirectX::SimpleMath::Vector2* vertices);
		bool pointOnLeft(unsigned int id) const;
		bool pointOnLeft(const NavMeshNode* node) const;
		friend class NavMesh;
	protected:
		DirectX::SimpleMath::Vector2 _point;
		DirectX::SimpleMath::Vector2 _dir;
		float _width;
		float _distance;
		NavMeshNode* _node0;
		NavMeshNode* _node1;
	};
}
