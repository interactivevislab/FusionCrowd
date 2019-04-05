#pragma once

#include <vector>

#include "NavSystem.h"
#include "Agent.h"
#include "IStrategyComponent.h"
#include "ITacticComponent.h"
#include "IOperComponent.h"
#include "Config.h"
#include "OperationComponent/SpatialQuery/SpatialQuery.h"
#include "Goal/PointGoal.h"

#include "NavComponents/NavMeshCompnent.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"

class FUSION_CROWD_API Simulator
{
public:
	Simulator();
	~Simulator();

	bool DoStep();
	void AddAgent(FusionCrowd::Agent agent);
	void AddAgent(float maxAngleVel, float maxNeighbors, int obstacleSet,
		float neighborDist, float radius, float prefSpeed, float maxSpeed, float maxAccel, FusionCrowd::Math::Vector2 pos);
	void AddOperComponent(IOperComponent* operComponent);
	void AddSpatialQuery(FusionCrowd::SpatialQuery* spatialQuery);
	void AddNavComponent(std::string name, INavComponent* navComponent);
	void ComputeNeighbors(FusionCrowd::Agent * agent);

	void InitSimulator(char* navMeshPath);

//private:
	NavSystem navSystem;
	std::vector<FusionCrowd::SpatialQuery*> spatialQuerys;
	std::vector<FusionCrowd::Agent> agents;
	std::vector<IStrategyComponent> strategyComponents;
	std::vector<ITacticComponent> tacticComponents;
	std::vector<IOperComponent*> operComponents;
	Goal* goal;
	NavMeshCompnent nav;
};

