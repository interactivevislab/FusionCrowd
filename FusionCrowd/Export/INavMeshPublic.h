#pragma once

#include "Config.h"
#include "Export/FCArray.h"

namespace FusionCrowd
{
	struct FUSION_CROWD_API NavMeshVetrex
	{
		float X, Y;
	};

	struct FUSION_CROWD_API EdgeInfo
	{
		//p0, p1
		float x1, x2, y1, y2;
		//nodes centers
		float nx0, nx1, ny0, ny1;
		int error_code;
	};

	class INavMeshPublic
	{
	public:
		virtual size_t GetVertexCount() = 0;
		virtual bool GetVertices(FCArray<NavMeshVetrex> & output) = 0;
		virtual size_t GetNodesCount() = 0;
		virtual size_t GetNodeVertexCount(size_t node_id) = 0;
		virtual bool GetNodeVertexInfo(FCArray<int> & output, size_t node_id) = 0;
		virtual size_t GetEdgesCount() = 0;
		virtual bool GetEdges(FCArray<EdgeInfo> & output) = 0;
		virtual size_t GetObstaclesCount() = 0;
		virtual bool GetObstacles(FCArray<EdgeInfo> & output) = 0;
		virtual bool ExportNavMeshToFile(char* file_name) = 0;
	};
}