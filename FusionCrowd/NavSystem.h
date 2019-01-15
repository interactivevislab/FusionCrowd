#pragma once

#include <map>
#include <string>

#include "INavComponent.h"

class NavSystem
{
public:
	NavSystem();

	void Update();

	~NavSystem();
private:
	std::map<std::string, INavComponent> navComponent;
};

