#include "NavSystem.h"



NavSystem::NavSystem(std::string name, INavComponent* namComponent)
{
	AddNavComponent(name, namComponent);
}

void NavSystem::AddNavComponent(std::string name, INavComponent* namComponent)
{
	navComponents.insert(std::pair<std::string, INavComponent*>(name, namComponent));
}

void NavSystem::Update()
{

}

NavSystem::~NavSystem()
{
}
