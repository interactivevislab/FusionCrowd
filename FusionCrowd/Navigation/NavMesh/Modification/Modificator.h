#pragma once

#include "Navigation/NavMesh/NavMesh.h"
#include "Math/Util.h"

#include <vector>

namespace FusionCrowd {
	enum ModificationTypes {
		SPLIT,
		CUT_POLY,
		CUT_CURVE
	};

	struct NodeModificator {
		NavMeshNode* node;
		std::vector<DirectX::SimpleMath::Vector2> polygon_to_cut;
		std::vector<unsigned int> polygon_vertex_ids;
		ModificationTypes modification_type;
		bool side;
		bool correct = true;
	};
 }