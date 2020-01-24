#pragma once
#include "../NavMesh.h"
#include <vector>

using namespace DirectX::SimpleMath;

namespace FusionCrowd {
	enum ModificationTypes {
		SPLIT,
		CUT_POLY,
		CUT_CURVE
	};

	struct NodeModificator {
		NavMeshNode* node;
		std::vector<Vector2> polygon_to_cut;
		std::vector<unsigned int> polygon_vertex_ids;
		ModificationTypes modification_type;
		bool side;
		bool correct = true;
	};
 }