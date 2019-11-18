#include "NavMeshNode.h"
#include "NavMeshObstacle.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMeshNode::NavMeshNode() : _edges(0x0), _edgeCount(0), _obstacles(0x0), _obstCount(0),
	                             _center(), _poly()
	{
	}

	NavMeshNode::~NavMeshNode()
	{
		if (_edges)
		{
			delete[] _edges;
		}
		if (_obstacles)
		{
			delete[] _obstacles;
		}
	}

	NavMeshNode& NavMeshNode::operator=(const NavMeshNode& n)
	{
		// Copy edges
		// If I already have memory and it is too small, delete and create
		if (_edgeCount < n._edgeCount)
		{
			if (_edgeCount > 0) delete[] _edges;
			_edges = new NavMeshEdge*[n._edgeCount];
		}

		_edgeCount = n._edgeCount;
		for (unsigned int e = 0; e < _edgeCount; ++e)
		{
			_edges[e] = n._edges[e];
		}

		// Copy obstacles
		if (_obstCount < n._obstCount)
		{
			if (_obstCount > 0) delete[] _edges;
			_obstacles = new NavMeshObstacle*[n._obstCount];
		}
		_obstCount = n._obstCount;
		for (unsigned int o = 0; o < _obstCount; ++o)
		{
			_obstacles[o] = n._obstacles[o];
		}

		_center = n._center;
		_poly = n._poly;
		_id = n._id;
		return *this;
	}

	const NavMeshNode* NavMeshNode::getNeighbor(size_t i) const
	{
		return _edges[i]->getOtherByPtr(this);
	}

	NavMeshEdge* NavMeshNode::getConnection(unsigned nodeID)
	{
		for (size_t e = 0; e < _edgeCount; ++e)
		{
			NavMeshEdge* edge = _edges[e];
			NavMeshNode* neighbor = edge->getOtherByPtr(this);
			if (neighbor->_id == nodeID)
			{
				return edge;
			}
		}
		return NULL;
	}

	NavMeshPoly NavMeshNode::getPoly() {
		return _poly;
	}

#ifdef _WIN32
	// This disables a 64-bit compatibility warning - pushing a 32-bit value into a 64-bit value.
	// This can cause problems with SIGN EXTENSION.
	// In this case, I know the value in being put into the pointer slot is an unsigned
	//	int, so sign extension is not a problem.  Plus, they never get interpreted as
	//	pointers.  These indices are eventually mapped to REAL pointers.
#pragma warning( disable : 4312 )
#endif
	bool NavMeshNode::loadFromAscii(std::istream& f)
	{
		// center
		float cx, cy;
		if ((f >> cx >> cy))
		{
			_center = Vector2(cx, cy);
		}
		else
		{
			return false;
		}

		// polygon
		if (!_poly.loadFromAscii(f))
		{
			return false;
		}

		// edges
		if (!(f >> _edgeCount))
		{
			return false;
		}
		_edges = new NavMeshEdge*[_edgeCount];
		for (size_t e = 0; e < _edgeCount; ++e)
		{
			size_t eID;
			if (!(f >> eID))
			{
				return false;
			}
			// This is a cheat -- I'm storing the index now, to convert it to a pointer later.
			_edges[e] = (NavMeshEdge *)eID;
		}

		// obstacles
		if (!(f >> _obstCount))
		{
			return false;
		}
		_obstacles = new NavMeshObstacle*[_obstCount];
		for (size_t o = 0; o < _obstCount; ++o)
		{
			size_t oID;
			if (!(f >> oID))
			{
				return false;
			}
			// This is a cheat -- I'm storing the index now, to convert it to a pointer later.
			_obstacles[o] = (NavMeshObstacle *)oID;
		}
		return true;
	}

	void NavMeshNode::setVertices(const DirectX::SimpleMath::Vector2* vertices)
	{
		_poly.SetVertices(vertices);
	}

}
#ifdef _WIN32
#pragma warning( default : 4312 )
#endif
