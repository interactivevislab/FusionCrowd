#include "SpatialQuery.h"


namespace FusionCrowd
{
	SpatialQuery::SpatialQuery(): _testVisibility(false)
	{
	}

	void SpatialQuery::AddObstacle(Obstacle *obs)
	{
		obs->_id = _obstacles.size();
		_obstacles.push_back(obs);
	}
}
