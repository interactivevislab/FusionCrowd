#pragma once

#include "Config.h"
#include "Export/FCArray.h"
#include "Export/INavMeshPublic.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API INavSystemPublic
	{
	public:
		virtual INavMeshPublic* GetPublicNavMesh() const = 0;

		virtual float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) = 0;
	};
}
