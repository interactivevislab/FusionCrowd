#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcFileWrapper.h"

#include <vector>


namespace FusionCrowdWeb
{
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
		float CenterX;

		/** Y coordinate of region's center. */
		float CenterY;

		/** Width of region. */
		float Width = -1;

		/** Height of region. */
		float Height = -1;

		NavMeshRegion() = default;
		NavMeshRegion(const std::string& inNavMeshPath);

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
		std::vector<NavMeshRegion> Split(size_t inNumParts);

	private:
		/**
		* \fn Split
		* \brief Splits region into several parts with equal area.
		*
		* @param inNumParts	Number of parts to split into.
		*/
		void Split(size_t inNumParts, std::vector<NavMeshRegion>& outParts);
	};


	/**
	* \struct InitComputingData
	* \brief Data for simulation initialization.
	*/
	struct FC_WEB_API InitComputingData
	{
		InitComputingData();
		InitComputingData(const std::string& inNavMeshFileName, FusionCrowd::FCArray<AgentInitData> inAgentsData);

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
		InputComputingData(float inTimeStep);

		/** Simulation step in seconds. */
		float TimeStep;

		/** Agents from other NavMeshRegions. */
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> NewAgents;

		/** Agents from NavMeshRegion's boundary zone. */
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> BoundaryAgents;
	};


	/**
	* \struct OutputComputingData
	* \brief Result data of simulation step.
	*/
	struct FC_WEB_API OutputComputingData
	{
		OutputComputingData();
		OutputComputingData(FusionCrowd::FCArray<FusionCrowd::AgentInfo> inAgentInfos);

		/** Data of agents in NavMeshRegion. */
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> AgentInfos;

		/** Data of agents that left NavMeshRegion. */
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> DisplacedAgents;
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
