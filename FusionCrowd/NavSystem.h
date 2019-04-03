#pragma once

#include <map>
#include <string>
#include "Config.h"

#include "NavComponents/INavComponent.h"

class FUSION_CROWD_API NavSystem
{
public:
	NavSystem() {}
	NavSystem(std::string name, INavComponent* namComponent);
	void AddNavComponent(std::string name, INavComponent* namComponent);
	~NavSystem();

	void Update();
private:
	std::map<std::string, INavComponent *> navComponents;
};

