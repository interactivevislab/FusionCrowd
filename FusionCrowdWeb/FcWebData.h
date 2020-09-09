#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcFileWrapper.h"


namespace FusionCrowdWeb
{
	/**
	* \struct ShortAgentInfo
	* \brief Lightweight version of FusionCrowd::AgentInfo for network transmission.
	*/
	struct FC_WEB_API ShortAgentInfo
	{
		ShortAgentInfo();
		ShortAgentInfo(FusionCrowd::AgentInfo inInfo);
		operator FusionCrowd::AgentInfo();

		size_t Id;

		/** X coordinate of agent's position. */
		float PosX;
		/** X coordinate of agent's position. */
		float PosY;

		/** X coordinate of agent's velocity. */
		float VelX;
		/** Y coordinate of agent's velocity. */
		float VelY;

		/** X coordinate of agent's orient. */
		float OrientX;
		/** Y coordinate of agent's orient. */
		float OrientY;

		/** X coordinate of agent's goal. */
		float GoalX;
		/** Y coordinate of agent's goal. */
		float GoalY;

		/** Server's id where the agent is from. */
		int ServerId;
	};

	/**
	* \struct AgentInitData
	* \brief Agent's initialization data for simulation.
	*/
	struct FC_WEB_API AgentInitData
	{
		/** X coordinate of agent's position. */
		float X;

		/** Y coordinate of agent's position. */
		float Y;

		/** X coordinate of agent's goal position. */
		float GoalX;

		/** Y coordinate of agent's goal position. */
		float GoalY;
	};


	/**
	* \struct NavMeshRegion
	* \brief Rectangular area of NavMesh.
	*/
	struct FC_WEB_API NavMeshRegion
	{
	public:
		/** X coordinate of region's center. */
		float CenterX = 0;

		/** Y coordinate of region's center. */
		float CenterY = 0;

		/** Width of region. */
		float Width = -1;

		/** Height of region. */
		float Height = -1;

		NavMeshRegion() = default;
		NavMeshRegion(const char* inNavMeshPath);

		/**
		* \fn IsPointInside
		* \brief Determines if point is inside region or not.
		*
		* @param inX	X coordinate of point.
		* @param inY	Y coordinate of point.
		*
		* @return Boolean value "point is inside or not".
		*/
		bool IsPointInside(float inX, float inY);

		/**
		* \fn IsPointInsideBoundaryZone
		* \brief Determines if point is in region's boundary zone.
		*
		* @param inX					X coordinate of point.
		* @param inY					Y coordinate of point.
		* @param inBoundaryZoneDepth	Boundary zone size as the distance from the region's border.
		*
		* @return Boolean value "point is inside or not".
		*/
		bool IsPointInsideBoundaryZone(float inX, float inY, float inBoundaryZoneDepth);

		/**
		* \fn Split
		* \brief Splits region into several parts with equal area.
		*
		* @param inNumParts	Number of parts to split into.
		*
		* @return Parts of region.
		*/
		FusionCrowd::FCArray<NavMeshRegion> Split(size_t inNumParts);

	private:
		/**
		* \fn Split
		* \brief Splits region into several parts with equal area.
		*
		* @param inNumParts		Number of parts to split into.
		* @param inNextIndex	Next index for part in array.
		*/
		void Split(size_t inNumParts, size_t& inNextIndex, FusionCrowd::FCArray<NavMeshRegion>& outParts);
	};


	/**
	* \struct ChangeGoalData
	* \brief Data for changing agent's goal.
	*/
	struct FC_WEB_API ChangeGoalData
	{
		/** Agent's id. */
		size_t AgentId;

		/** X coordinate of agent's new goal position. */
		float NewGoalX;

		/** Y coordinate of agent's new goal position. */
		float NewGoalY;
	};

	
	/**
	* \struct InitComputingData
	* \brief Data for simulation initialization.
	*/
	struct FC_WEB_API InitComputingData
	{
		InitComputingData();
		InitComputingData(const char* inNavMeshFileName, const FusionCrowd::FCArray<AgentInitData>& inAgentsData);

		/** Wrapped file with navigation mesh. */
		FcFileWrapper NavMeshFile;

		/** Agents' initialization data. */
		FusionCrowd::FCArray<AgentInitData> AgentsData;

		/** Region of navigation mesh for simulation. */
		NavMeshRegion NavMeshRegion;
	};


	/**
	* \struct InputComputingData
	* \brief Data for simulation step.
	*/
	struct FC_WEB_API InputComputingData
	{
		InputComputingData();
		InputComputingData(float inTimeStep, const FusionCrowd::FCArray<ChangeGoalData>& inAgentsNewGoals);

		/** Simulation step in seconds. */
		float TimeStep = 0.1f;

		/** Agents from other NavMeshRegions. */
		FusionCrowd::FCArray<ShortAgentInfo> NewAgents;

		/** Agents from NavMeshRegion's boundary zone. */
		FusionCrowd::FCArray<ShortAgentInfo> BoundaryAgents;

		/** Agents from NavMeshRegion's boundary zone. */
		FusionCrowd::FCArray<ChangeGoalData> NewAgentsGoals;
	};


	/**
	* \struct OutputComputingData
	* \brief Result data of simulation step.
	*/
	struct FC_WEB_API OutputComputingData
	{
		OutputComputingData();
		OutputComputingData(FusionCrowd::FCArray<ShortAgentInfo> inAgentInfos);

		/** Data of agents in NavMeshRegion. */
		FusionCrowd::FCArray<ShortAgentInfo> AgentInfos;

		/** Data of agents that left NavMeshRegion. */
		FusionCrowd::FCArray<ShortAgentInfo> DisplacedAgents;
	};


	/**
	* \struct AgentsIds
	* \brief Wrapper for array of agents' ids.
	*/
	struct FC_WEB_API AgentsIds
	{
		AgentsIds();
		AgentsIds(size_t inNum);

		/** Inner array of agents' ids. */
		FusionCrowd::FCArray<size_t> Values;
	};
}
