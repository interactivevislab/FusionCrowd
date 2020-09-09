#include "FcWebData.h"

#include "Export/INavMeshPublic.h"


namespace FusionCrowdWeb
{
	ShortAgentInfo::ShortAgentInfo()
	{
	}


	ShortAgentInfo::ShortAgentInfo(FusionCrowd::AgentInfo inInfo)
	{
		Id			= inInfo.id;
		PosX		= inInfo.posX;
		PosY		= inInfo.posY;
		VelX		= inInfo.velX;
		VelY		= inInfo.velY;
		OrientX		= inInfo.orientX;
		OrientY		= inInfo.orientY;
		GoalX		= inInfo.goalX;
		GoalY		= inInfo.goalY;
		ServerId	= inInfo.serverId;
	}


	ShortAgentInfo::operator FusionCrowd::AgentInfo()
	{
		FusionCrowd::AgentInfo info;
		info.id			= Id;
		info.posX		= PosX;
		info.posY		= PosY;
		info.velX		= VelX;
		info.velY		= VelY;
		info.orientX	= OrientX;
		info.orientY	= OrientY;
		info.goalX		= GoalX;
		info.goalY		= GoalY;
		info.serverId	= ServerId;

		return info;
	}


	NavMeshRegion::NavMeshRegion(const char* inNavMeshPath)
	{
		using namespace FusionCrowd;

		auto vertices = NavMeshHelper::LoadNavMeshVertices(inNavMeshPath);

		if (vertices.size() == 0)
		{
			return;
		}

		float maxX, minX, maxY, minY;
		maxX = minX = vertices[0].X;
		maxY = minY = vertices[0].Y;

		for (auto vertex : vertices)
		{
			maxX = vertex.X > maxX ? vertex.X : maxX;
			minX = vertex.X < minX ? vertex.X : minX;
			maxY = vertex.Y > maxY ? vertex.Y : maxY;
			minY = vertex.Y < minY ? vertex.Y : minY;
		}

		CenterX	= (maxX + minX) / 2;
		CenterY	= (maxY + minY) / 2;
		Width	= maxX - minX;
		Height	= maxY - minY;
	}


	bool NavMeshRegion::IsPointInside(float inX, float inY)
	{
		return (inX > CenterX - Width / 2)
			&& (inX < CenterX + Width / 2)
			&& (inY > CenterY - Height / 2)
			&& (inY < CenterY + Height / 2);
	}


	bool NavMeshRegion::IsPointInsideBoundaryZone(float inX, float inY, float inBoundaryZoneDepth)
	{
		auto regionCopy = *this;
		regionCopy.Width	+= inBoundaryZoneDepth;
		regionCopy.Height	+= inBoundaryZoneDepth;
		return (regionCopy.IsPointInside(inX, inY)) && (!IsPointInside(inX, inY));
	}


	void NavMeshRegion::Split(size_t inNumParts, size_t& inNextIndex, FusionCrowd::FCArray<NavMeshRegion>& outParts)
	{
		if (inNumParts == 1)
		{
			outParts[inNextIndex++] = *this;
			return;
		}

		NavMeshRegion part1, part2;
		float share = static_cast<float>(inNumParts / 2) / inNumParts;
		if (Width > Height)
		{
			part1.CenterY	= CenterY;
			part2.CenterY	= CenterY;
			part1.Height	= Height;
			part2.Height	= Height;

			part1.Width = share * Width;
			part2.Width = Width - part1.Width;

			part1.CenterX = CenterX - part1.Width / 2;
			part2.CenterX = CenterX + part2.Width / 2;
		}
		else
		{
			part1.CenterX	= CenterX;
			part2.CenterX	= CenterX;
			part1.Width		= Width;
			part2.Width		= Width;

			part1.Height = share * Height;
			part2.Height = Height - part1.Height;

			part1.CenterY = CenterY - part1.Height / 2;
			part2.CenterY = CenterY + part2.Height / 2;
		}

		part1.Split(inNumParts / 2, inNextIndex, outParts);
		part2.Split(inNumParts - inNumParts / 2, inNextIndex, outParts);
	}


	FusionCrowd::FCArray<NavMeshRegion> NavMeshRegion::Split(size_t inNumParts)
	{
		FusionCrowd::FCArray<NavMeshRegion> navMeshRegionsBuffer(inNumParts);
		size_t indexBuffer = 0;
		Split(inNumParts, indexBuffer, navMeshRegionsBuffer);
		return navMeshRegionsBuffer;
	}


	InitComputingData::InitComputingData() : AgentsData(FusionCrowd::FCArray<AgentInitData>(0))
	{
	}


	InitComputingData::InitComputingData(const char* inNavMeshFileName,
		const FusionCrowd::FCArray<AgentInitData>& inAgentsData)
		: NavMeshFile(inNavMeshFileName), AgentsData(inAgentsData)
	{
	}


	InputComputingData::InputComputingData()
		: NewAgents(FusionCrowd::FCArray<ShortAgentInfo>(0)),
		BoundaryAgents(FusionCrowd::FCArray<ShortAgentInfo>(0)),
		NewAgentsGoals(FusionCrowd::FCArray<ChangeGoalData>(0))
	{
	}


	InputComputingData::InputComputingData(float inTimeStep, const FusionCrowd::FCArray<ChangeGoalData>& inAgentsNewGoals)
		: TimeStep(inTimeStep), NewAgents(FusionCrowd::FCArray<ShortAgentInfo>(0)),
		BoundaryAgents(FusionCrowd::FCArray<ShortAgentInfo>(0)),
		NewAgentsGoals(inAgentsNewGoals)
	{
	}


	OutputComputingData::OutputComputingData()
		: AgentInfos(FusionCrowd::FCArray<ShortAgentInfo>(0)),
		DisplacedAgents(FusionCrowd::FCArray<ShortAgentInfo>(0))
	{
	}


	OutputComputingData::OutputComputingData(FusionCrowd::FCArray<ShortAgentInfo> inAgentInfos)
		: AgentInfos(inAgentInfos), DisplacedAgents(FusionCrowd::FCArray<ShortAgentInfo>(0))
	{
	}


	AgentsIds::AgentsIds() : Values(FusionCrowd::FCArray<size_t>(0))
	{
	}


	AgentsIds::AgentsIds(size_t inNum) : Values(FusionCrowd::FCArray<size_t>(inNum))
	{
	}
}
