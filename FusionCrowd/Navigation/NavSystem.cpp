#include "NavSystem.h"

namespace FusionCrowd
{
	NavSystem::NavSystem()
	{
	}

	void NavSystem::AddNavComponent(std::string name, INavComponent* namComponent)
	{
		_navComponents.insert({name, namComponent});
	}

	INavComponent * NavSystem::GetNavComponent(std::string name)
	{
		return nullptr;
	}

	void NavSystem::AddAgent(AgentSpatialInfo spatialInfo, INavComponent * navComponent)
	{
	}

	std::vector<Agent> NavSystem::GetNeighbours(FusionCrowd::Agent & agent) const
	{
		return std::vector<Agent>();
	}

	std::vector<Obstacle> NavSystem::GetClosestObstacles(FusionCrowd::Agent & agent) const
	{
		return std::vector<Obstacle>();
	}

	void NavSystem::AddSpatialQuery(FusionCrowd::SpatialQuery * spatialQuery)
	{
	}

	void NavSystem::Update(float timeStep)
	{
		for (const auto& pair : _navComponents)
		{
			INavComponent* component = pair.second;
		}
	}

	NavSystem::~NavSystem()
	{
	}
}
