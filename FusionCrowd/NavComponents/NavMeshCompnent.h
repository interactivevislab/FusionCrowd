#pragma once

#include <string>

#include "INavComponent.h"
#include "NavMesh/NavMesh.h"


class NavMeshCompnent :
	public INavComponent
{
public:
	NavMeshCompnent();

	void SetPrefVelocity();

	~NavMeshCompnent();

private:
	NavMesh navMesh;
};

