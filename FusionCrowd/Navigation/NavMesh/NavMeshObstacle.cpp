#include "NavMeshObstacle.h"

namespace FusionCrowd
{
	const float MIN_EDGE_WIDTH = 0.9e-7f;

#pragma push_macro("max")
#undef max

	size_t NavMeshObstacle::NO_NEIGHBOR_OBST = std::numeric_limits<size_t>::max();

#pragma pop_macro("max")

#ifdef _WIN32
	// This disables a 64-bit compatibility warning - pushing a 32-bit value into a 64-bit value.
	// This can cause problems with SIGN EXTENSION.
	// In this case, I know the value in being put into the pointer slot is an unsigned
	//	int, so sign extension is not a problem.  Plus, they never get interpreted as
	//	pointers.  These indices are eventually mapped to REAL pointers.
#pragma warning( disable : 4312 )
#endif

	using namespace DirectX::SimpleMath;

	bool NavMeshObstacle::LoadFromAscii(std::istream& f, Vector2* vertices)
	{
		size_t v0, v1, node;
		long int nextObst;
		if (!(f >> v0 >> v1 >> node >> nextObst))
		{
			return false;
		}
		else
		{
			_point = vertices[v0];
			Vector2 disp = vertices[v1] - vertices[v0];
			_length = disp.Length();
			if (_length <= MIN_EDGE_WIDTH)
			{
				return false;
			}
			_unitDir = disp / _length;
			// Stash indices as pointers
			if (nextObst >= 0)
			{
				_nextObstacle = (Obstacle *)nextObst;
			}
			else
			{
				_nextObstacle = (Obstacle *)NO_NEIGHBOR_OBST;
			}
			_node = (NavMeshNode *)node;
		}
		return true;
	}
#ifdef _WIN32
#pragma warning( default : 4312 )
#endif

	NavMeshObstacle::~NavMeshObstacle()
	{
	}
}
