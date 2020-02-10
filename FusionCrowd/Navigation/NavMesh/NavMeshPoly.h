#pragma once

#include <fstream>
#include <string>

#include "Math/Util.h"
#include "Math/BoundingBox.h"

namespace FusionCrowd
{
	class NavMeshPoly
	{
	public:
		NavMeshPoly();
		~NavMeshPoly();

		NavMeshPoly(const NavMeshPoly& p);

		NavMeshPoly& operator=(const NavMeshPoly& n);
		bool containsPoint(const DirectX::SimpleMath::Vector2& point) const;
		float getElevation(const DirectX::SimpleMath::Vector2& point) const;
		DirectX::SimpleMath::Vector2 getGradient() const { return DirectX::SimpleMath::Vector2(A, B); }
		DirectX::SimpleMath::Vector2 getVertexByPos(int pos) const { return vertices[vertIDs[pos]]; }
		bool loadFromAscii(std::istream& f);
		bool loadFromBinary(std::istream& f);
		void initialize(size_t vCount, unsigned int* ids, float A = 0.f, float B = 0.f,
		                float C = 0.f);

		unsigned int* vertIDs;
		size_t vertCount;
		const DirectX::SimpleMath::Vector2* vertices;

		void SetVertices(const DirectX::SimpleMath::Vector2* vertices);

		void setBB();
		const BoundingBox & getBB() const;

		float A;
		float B;
		float C;
	private:
		BoundingBox _box;
	};
}
