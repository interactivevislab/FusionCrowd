#pragma once

#include <vector>

#include "Agent.h"
#include "Config.h"

#include "StrategyComponent/IStrategyComponent.h"
#include "StrategyComponent/Goal/PointGoal.h"

#include "TacticComponent/NavMeshComponent.h"
#include "TacticComponent/ITacticComponent.h"

#include "OperationComponent/IOperationComponent.h"

#include "Navigation/NavSystem.h"
#include "Navigation/SpatialQuery/SpatialQuery.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API Simulator
	{
	public:
		Simulator();
		~Simulator();

		bool DoStep();

		FusionCrowd::Agent* getById(size_t id);

		void AddAgent(FusionCrowd::Agent agent);
	    void AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos);
		//void AddAgent(float maxAngleVel, float maxNeighbors, int obstacleSet, float neighborDist, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos);

		void AddOperComponent(IOperationComponent & operComponent);
		void AddTacticComponent(ITacticComponent & tacticComponent);
		void AddStrategyComponent(IStrategyComponent & strategyComponent);

		void InitSimulator();

	//private:
		NavSystem navSystem;

		//std::vector<FusionCrowd::SpatialQuery*> spatialQuerys;


		std::vector<FusionCrowd::Agent> agents;

		std::vector<std::reference_wrapper<IStrategyComponent>> strategyComponents;
		std::vector<std::reference_wrapper<ITacticComponent>> tacticComponents;
		std::vector<std::reference_wrapper<IOperationComponent>> operComponents;
	};
}
