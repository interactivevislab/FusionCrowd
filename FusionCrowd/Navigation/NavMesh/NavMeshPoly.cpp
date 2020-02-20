#include "NavMeshPoly.h"

using std::memcpy;
using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshPoly::NavMeshPoly() : vertIDs(0x0), vertCount(0), A(0.f), B(0.f), C(0.f)
	{
	}

	NavMeshPoly::NavMeshPoly(const NavMeshPoly& p)
		: vertIDs(new unsigned int[p.vertCount]), vertCount(p.vertCount), vertices(p.vertices), A(p.A), B(p.B), C(p.C)
	{
		memcpy(&vertIDs[0], &p.vertIDs[0], vertCount * sizeof(unsigned int));
	}


	NavMeshPoly::~NavMeshPoly()
	{
		if (vertIDs)
		{
			delete[] vertIDs;
		}
	}

	NavMeshPoly& NavMeshPoly::operator=(const NavMeshPoly& p)
	{
		vertCount = p.vertCount;
		if (vertIDs) delete[] vertIDs;
		vertIDs = new unsigned int[vertCount];
		memcpy(&vertIDs[0], &p.vertIDs[0], vertCount * sizeof(unsigned int));
		vertices = p.vertices;
		_box = {p._box.xmin, p._box.ymin, p._box.xmax, p._box.ymax};
		A = p.A;
		B = p.B;
		C = p.C;
		return *this;
	}

	bool NavMeshPoly::containsPoint(const Vector2& point) const
	{
		const float X = point.x;
		const float Y = point.y;
		if(!_box.Contains(X, Y))
		{
			return false;
		}
		int count = 0; // number of intersections
		for (size_t e = 0; e < vertCount; ++e)
		{
			const Vector2& p0 = vertices[vertIDs[e]];

			if (p0.y == Y && p0.x <= X)
			{
				// There is a special case here where the line passes through the point
				//	tangentially to the polygon (i.e. it doesn't cut into the polygon.
				//
				//	   a\    /b
				//       \  /
				//       c\/______x
				//
				// The line segment through the test point x intersects point c, but the
				//	line segment doesn't cut through the polygon.  Counting this would be
				//	incorrect.
				//
				// The solution is to test points a and b and make sure they lie on opposite
				//	sides of the line through x.  If they do, it counts
				//
				// However if the test point IS the vertex, then it DOES count.
				if (p0.x == X)
				{
					// the point is in the polygon
					//	this is slightly fragile -- the point will register as inside any
					//	polygon built on this point.
					//
					// A similar problem exists if x lies on the boundary of the polygon -- both
					//	 polygons will consider it to be "inside".
					return true;
				}
				else
				{
					size_t prev = e == 0 ? (vertCount - 1) : (e - 1);
					size_t next = e == (vertCount - 1) ? 0 : (e + 1);
					float pY = vertices[vertIDs[prev]].y;
					float nY = vertices[vertIDs[next]].y;
					// if both y-values lie on the same side of the line, it is incidental contact.
					//	Don't count the contact
					//	There can be problems with signed zero values.  Otherwise, probably safe.
					if ((pY > Y && nY > Y) || (pY < Y && nY < Y)) continue;
				}
				// If the line segment intersects the first point, count it and move on.
				++count;
				continue;
			}

			const size_t next = (e + 1) % vertCount;
			const Vector2& p1 = vertices[vertIDs[next]];
			// simple cases in which there can be no intersection
			if ((p0.y > Y && p1.y >= Y) || // polysegment above line
				(p0.y < Y && p1.y <= Y) || // polysegment below line
				(p0.x > X && p1.x > X))
			{
				// polysegment to right of test line

				continue;
			}
			// legitimate intersection test
			// compute where, between y0 and y1, I'll find Y
			float t = Y - p0.y;
			float x0 = p0.x;
			float dx = p1.x - x0;

			t /= p1.y - p0.y;
			float x = x0 + t * dx;
			if (x <= X)
			{
				// this includes if (X,Y) lies on the line between the two vertices
				++count;
			}
		}
		return (count & 0x1) == 1;
	}

	float NavMeshPoly::getElevation(const Vector2& point) const
	{
		return A * point.x + B * point.y + C;
	}

	bool NavMeshPoly::loadFromAscii(std::istream& f)
	{
		if (!(f >> vertCount))
		{
			return false;
		}
		if (vertIDs) delete[] vertIDs;
		vertIDs = new unsigned int[vertCount];
		for (size_t i = 0; i < vertCount; ++i)
		{
			if (!(f >> vertIDs[i]))
			{
				return false;
			}
		}
		if (!(f >> A >> B >> C))
		{
			return false;
		}

		return true;
	}

	bool NavMeshPoly::loadFromBinary(std::istream& f)
	{
		// TODO: This can lead to problems.  If the size of size_t changes,
		//	but the file spec doesn't, this will read the wrong amount of data.
		unsigned int data;
		f.read((char*)&data, sizeof(int));
		vertCount = static_cast<size_t>(data);
		if (f.fail())
		{
			return false;
		}
		if (vertIDs) delete[] vertIDs;
		vertIDs = new unsigned int[vertCount];
		f.read((char*)&vertIDs[0],
		       static_cast<std::streamsize>(vertCount) * sizeof(unsigned int));
		if (f.fail())
		{
			return false;
		}
		const int FLOAT_COUNT = 3;
		float fData[FLOAT_COUNT];
		f.read((char*)&fData[0], FLOAT_COUNT * sizeof(float));
		if (f.fail())
		{
			return false;
		}
		A = fData[0];
		B = fData[1];
		C = fData[2];
		return true;
	}

	void NavMeshPoly::initialize(size_t vCount, unsigned int* ids, float newA, float newB, float newC)
	{
		vertCount = vCount;
		if (vertIDs)
		{
			delete[] vertIDs;
		}
		vertIDs = new unsigned int[vCount];
		memcpy(vertIDs, ids, sizeof(unsigned int) * vCount);
		A = newA;
		B = newB;
		C = newC;
	}

	void NavMeshPoly::setBB()
	{
		float xmin = 1e6f;
		float ymin = 1e6f;
		float xmax = -1e6f;
		float ymax = -1e6f;

		for (size_t v = 0; v < vertCount; ++v)
		{
			const Vector2& p0 = vertices[vertIDs[v]];
			if (p0.x < xmin) xmin = p0.x;
			if (p0.x > xmax) xmax = p0.x;
			if (p0.y < ymin) ymin = p0.y;
			if (p0.y > ymax) ymax = p0.y;
		}

		_box = {xmin, ymin, xmax, ymax};
	}

	const BoundingBox& NavMeshPoly::getBB() const
	{
		return _box;
	}

	void NavMeshPoly::SetVertices(const Vector2* vertices)
	{
		this->vertices = vertices;
	}
}
