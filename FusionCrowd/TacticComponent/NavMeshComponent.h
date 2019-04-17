#pragma once

#include <string>

#include "ITacticComponent.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Agent.h"
#include "Config.h"

class FUSION_CROWD_API NavMeshComponent :
	public ITacticComponent
{
public:
	NavMeshComponent();

	void SetPrefVelocity(const FusionCrowd::Agent* agent,const Goal * goal,Agents::PrefVelocity & pVel);

	~NavMeshComponent();

	float	_headingDevCos;

//private:
	NavMeshPtr	_navMesh;
	NavMeshLocalizerPtr _localizer;
};

