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
