#pragma once

#include <string>

#include "INavComponent.h"
#include "NavMesh/NavMesh.h"
#include "NavMesh/NavMeshLocalizer.h"
#include "../Path/PrefVelocity.h"
#include "../Goal/Goal.h"
#include "../Agent.h"
#include "../Config.h"

class FUSION_CROWD_API NavMeshCompnent :
	public INavComponent
{
public:
	NavMeshCompnent();

	void SetPrefVelocity(const FusionCrowd::Agent* agent,const Goal * goal,Agents::PrefVelocity & pVel);

	~NavMeshCompnent();

	float	_headingDevCos;

//private:
	NavMeshPtr	_navMesh;
	NavMeshLocalizerPtr _localizer;
};

