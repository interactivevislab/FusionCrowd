#include "NavMeshModifyer.h"


using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	NavMeshModifyer::NavMeshModifyer(NavMesh&  navmesh) : _navmesh(navmesh)
	{
	}


	NavMeshModifyer::~NavMeshModifyer()
	{
	}

	float NavMeshModifyer::CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
		_current_poly = &_navmesh.nodes[0]._poly;
		Initialize(polygon);
		//split node
		//int res = SplitNode(Vector2(polygon[0].X, polygon[0].Y), Vector2(polygon[1].X, polygon[1].Y));

		//Cut poly from node
		FillAddedVertices();
		int res = CutPolyFromCurrentNode();

		//Cut curve
		/*
		FillAddedVertices();
		int res = CutCurveFromCurrentNode();
		*/
		Finalize();
		return res;
	}

	int NavMeshModifyer::SplitNode(Vector2 v0, Vector2 v1) {
		//TODO remove init
		addednodes_count = 2;
		delete[] addednodes;
		addednodes = new NavMeshNode[2];
		delete[] addedvertices;
		addedvertices_count = 2;
		addedvertices = new Vector2[2];
		if (v0.x > v1.x) {
			addedvertices[0] = polygonvertices[1];
			addedvertices[1] = polygonvertices[0];
		}
		else {
			addedvertices[0] = polygonvertices[0];
			addedvertices[1] = polygonvertices[1];
		}


		NavMeshNode node0 = NavMeshNode(), node1 = NavMeshNode();
		int n0size = 2, n1size = 2;
		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			if (IsPointUnderLine(v0, v1, vertex))
				n0size++;
			else
				n1size++;
		}
		node0.setID(_navmesh.nCount);
		node1.setID(_navmesh.nCount + 1);
		node0._poly.vertCount = n0size;
		node0._poly.vertIDs = new unsigned int[n0size];
		node1._poly.vertCount = n1size;
		node1._poly.vertIDs = new unsigned int[n1size];

		int added0 = 0, added1 = 0;
		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			if (IsPointUnderLine(v0, v1, vertex)) {
				node0._poly.vertIDs[added0] = _current_poly->vertIDs[i];
				added0++;
			}
			else {
				node1._poly.vertIDs[added1] = _current_poly->vertIDs[i];
				added1++;
			}
		}
		bool addingque = false;// v0.x > v1.x;
		if (v0.x == v1.x) {
			addingque = v0.y > v1.y;
		}
		if (addingque) {
			node1._poly.vertIDs[added1] = _navmesh.vCount;
			node1._poly.vertIDs[added1 + 1] = _navmesh.vCount + 1;
			node0._poly.vertIDs[added0 + 1] = _navmesh.vCount;
			node0._poly.vertIDs[added0] = _navmesh.vCount + 1;
		} else {
			node1._poly.vertIDs[added1 + 1] = _navmesh.vCount;
			node1._poly.vertIDs[added1] = _navmesh.vCount + 1;
			node0._poly.vertIDs[added0] = _navmesh.vCount;
			node0._poly.vertIDs[added0 + 1] = _navmesh.vCount + 1;
		}
		addednodes[0] = node0;
		addednodes[1] = node1;
		return 0;
	}

	/*Adds all created nodes and vertexes*/
	int NavMeshModifyer::Finalize() {
		//add created vertices
		Vector2* updvertices = new Vector2[_navmesh.vCount + addedvertices_count];
		for (int i = 0; i < _navmesh.vCount; i++) {
			updvertices[i] = _navmesh.vertices[i];
		}
		delete[] _navmesh.vertices;
		for (int i = 0; i < addedvertices_count; i++) {
			updvertices[_navmesh.vCount + i] = addedvertices[i];
		}
		_navmesh.vCount = _navmesh.vCount + addedvertices_count;
		_navmesh.vertices = updvertices;

		for (int i = 0; i < addednodes_count; i++) {
			addednodes[i].setVertices(_navmesh.vertices);
		}

		NavMeshNode* tmpNodes = new NavMeshNode[_navmesh.nCount + addednodes_count];
		for (size_t i = 0; i < _navmesh.nCount; i++)
		{
			tmpNodes[i] = _navmesh.nodes[i];
		}
		delete[] _navmesh.nodes;
		for (int i = 0; i < addednodes_count; i++) {
			tmpNodes[_navmesh.nCount+ i] = addednodes[i];
		}
		_navmesh.nCount += addednodes_count;
		_navmesh.nodes = tmpNodes;
		delete[] polygonvertices;
		delete[] addednodes;
		delete[] addedvertices;
		delete[] crosspoints_prev_vertex_ids;
		return 0;
	}

	/*Creates arrays*/
	int NavMeshModifyer::Initialize(FCArray<NavMeshVetrex> & polygon) {
		polygonvertices_count = polygon.size();
		polygonvertices = new Vector2[polygonvertices_count];
		if (!IsClockwise(polygon)) {
			for (int i = 0; i < polygonvertices_count; i++) {
				Vector2 v = Vector2(polygon[i].X, polygon[i].Y);
				polygonvertices[i] = v;
			}
		} else {
			for (int i = 0; i < polygonvertices_count; i++) {
				Vector2 v = Vector2(polygon[i].X, polygon[i].Y);
				polygonvertices[polygonvertices_count - i - 1] = v;
			}
		}

		addednodes_count = polygonvertices_count;
		addednodes = new NavMeshNode[addednodes_count];

		addedvertices_count = polygonvertices_count*2;
		addedvertices = new Vector2[addedvertices_count];

		crosspoints_prev_vertex_ids_num = polygonvertices_count;
		crosspoints_prev_vertex_ids = new int[polygonvertices_count];
		return 0;
	}

	/*Adds additional nodes*/
	int NavMeshModifyer::CutPolyFromCurrentNode() {
		for (int j = 0; j < polygonvertices_count; j++) {
			NavMeshNode updnode = NavMeshNode();
			updnode.setID(_navmesh.nCount + j);
			int vert_count = 0;
			Vector2 j0vert = polygonvertices[j];
			Vector2 j1vert = polygonvertices[(j + 1) % polygonvertices_count];
			Vector2 j2vert = polygonvertices[(j + 2) % polygonvertices_count];
			bool node_side0 = !IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			vert_count += 4;
			updnode._poly.vertCount = vert_count;
			updnode._poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;

			for (int i = 0; i < _current_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
				bool vertex_added = false;
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					updnode._poly.vertIDs[addedids] = _current_poly->vertIDs[i];
					addedids++;
					vertex_added = true;
				}
				if (i == crosspoints_prev_vertex_ids[j]) {
					if (vertex_added) {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j; //j0j1 crosspoint
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count); //polygon node j1
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 2) % polygonvertices_count); //polygon node j2
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + ((j + 1) % polygonvertices_count); //j1j2 crosspoint
						addedids++;
					}
					else {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j; //j0j1 crosspoint
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count); //polygon node j1
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 2) % polygonvertices_count); //polygon node j2
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + ((j + 1) % polygonvertices_count); //j1j2 crosspoint
						addedids++;
					}
				}
			}

			addednodes[j] = updnode;
		}
		return 0;
	}

	int NavMeshModifyer::CutCurveFromCurrentNode() {
		for (int j = 0; j < polygonvertices_count-2; j++) {
			NavMeshNode updnode = NavMeshNode();
			updnode.setID(_navmesh.nCount + j);
			int vert_count = 0;
			Vector2 j0vert = polygonvertices[j];
			Vector2 j1vert = polygonvertices[(j + 1) % polygonvertices_count];
			Vector2 j2vert = polygonvertices[(j + 2) % polygonvertices_count];
			bool node_side0 = !IsPointUnderLine(j0vert, j1vert, j2vert);
			bool node_side1 = IsPointUnderLine(j1vert, j2vert, j0vert);
			for (int i = 0; i < _current_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					vert_count++;
				}
			}
			vert_count += 4;
			updnode._poly.vertCount = vert_count;
			updnode._poly.vertIDs = new unsigned int[vert_count];

			int addedids = 0;

			for (int i = 0; i < _current_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
				bool vertex_added = false;
				if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true) &&
					IsPointUnderLine(j1vert, j2vert, vertex, node_side1)) {
					updnode._poly.vertIDs[addedids] = _current_poly->vertIDs[i];
					addedids++;
					vertex_added = true;
				}
				if (i == crosspoints_prev_vertex_ids[j]) {
					if (vertex_added) {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j; //j0j1 crosspoint
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count); //polygon node j1
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 2) % polygonvertices_count); //polygon node j2
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + ((j + 1) % polygonvertices_count); //j1j2 crosspoint
						addedids++;
					}
					else {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j; //j0j1 crosspoint
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count); //polygon node j1
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 2) % polygonvertices_count); //polygon node j2
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + ((j + 1) % polygonvertices_count); //j1j2 crosspoint
						addedids++;
					}
				}
			}

			addednodes[j] = updnode;
		}

		//add node splited by v0 v1 line

		NavMeshNode updnode = NavMeshNode();
		updnode.setID(_navmesh.nCount + polygonvertices_count - 2);
		int vert_count = 0;
		Vector2 j0vert = polygonvertices[0];
		Vector2 j1vert = polygonvertices[1];
		Vector2 j2vert = polygonvertices[2];
		bool node_side0 = IsPointUnderLine(j0vert, j1vert, j2vert);
		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				vert_count++;
			}
		}
		vert_count += 2;
		updnode._poly.vertCount = vert_count;
		updnode._poly.vertIDs = new unsigned int[vert_count];

		int addedids = 0;

		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			if (IsPointUnderLine(j0vert, j1vert, vertex, node_side0, true)) {
				updnode._poly.vertIDs[addedids] = _current_poly->vertIDs[i];
				addedids++;
			}
			if (i == crosspoints_prev_vertex_ids[polygonvertices_count - 1]) {
				updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count; //j0j1 crosspoint
				addedids++;
				updnode._poly.vertIDs[addedids] = _navmesh.vCount; //polygon node j0
				addedids++;
				updnode._poly.vertIDs[addedids] = _navmesh.vCount + 1; //polygon node j0
				addedids++;
			}
		}
		addednodes[polygonvertices_count - 2] = updnode;

		addednodes_count--;
		return 0;
	}

	/*Returns cross point with current_poly in dirrection v0->v1*/
	Vector2 NavMeshModifyer::FindCrossPoint(Vector2 v0, Vector2 v1, int& out_prev_cross_id) {
		//calculate line coef
		float k0 = 0.0;
		float c0 = 0.0;
		bool vertical = false;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k0 = (v0.y - v1.y) / (v0.x - v1.x);
			c0 = v0.y - k0 * v0.x;
		}
		for (int i = 0; i < _current_poly->vertCount; i++) {
			Vector2 pnode0 = Vector2();
			Vector2 pnode1 = Vector2();
			if (i < _current_poly->vertCount - 1) {
				pnode0.x = _current_poly->vertices[_current_poly->vertIDs[i]].x;
				pnode0.y = _current_poly->vertices[_current_poly->vertIDs[i]].y;
				pnode1.x = _current_poly->vertices[_current_poly->vertIDs[i + 1]].x;
				pnode1.y = _current_poly->vertices[_current_poly->vertIDs[i + 1]].y;
			}
			else {
				pnode0.x = _current_poly->vertices[_current_poly->vertIDs[i]].x;
				pnode0.y = _current_poly->vertices[_current_poly->vertIDs[i]].y;
				pnode1.x = _current_poly->vertices[_current_poly->vertIDs[0]].x;
				pnode1.y = _current_poly->vertices[_current_poly->vertIDs[0]].y;
			}

			//line cross point claculation
			float xcross = 0.0;
			float ycross = 0.0;
			if (vertical) {
				if (pnode0.x == pnode1.x) continue;
				float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
				float c1 = pnode0.y - k1 * pnode0.x;
				xcross = v0.x;
				ycross = k1 * xcross + c1;
			} else {
				if (pnode0.x == pnode1.x) {
					xcross = pnode0.x;
					ycross = k0 * xcross + c0;
				}
				else {
					float k1 = (pnode0.y - pnode1.y) / (pnode0.x - pnode1.x);
					if (k1 == k0) continue;
					float c1 = pnode0.y - k1 * pnode0.x;

					xcross = (c1 - c0) / (k0 - k1);
					ycross = k1 * xcross + c1;
				}
			}

			//is cross point on segment?
			if (
				((xcross>=pnode0.x && xcross<=pnode1.x) ||
				(xcross<=pnode0.x && xcross>=pnode1.x)) &&
				((ycross >= pnode0.y && ycross <= pnode1.y) ||
				(ycross <= pnode0.y && ycross >= pnode1.y))
				) {
				//is cross point in v0->v1 direction?
				if (
					(v1.x > v0.x && xcross >= v1.x) ||
					(v1.x < v0.x && xcross <= v1.x) ||
					(v1.x == v0.x && v1.y> v0.y && ycross >= v1.y) ||
					(v1.x == v0.x && v1.y < v0.y && ycross <= v1.y)
					){
					out_prev_cross_id = i;
					return Vector2(xcross, ycross);
				}
				else continue;
			}
		}
		return Vector2(0, 0);
	}

	/*Adds crosspoints for polygon cut*/
	void NavMeshModifyer::FillAddedVertices() {
		for (int i = 0; i < polygonvertices_count; i++) {
			addedvertices[i] = polygonvertices[i];
		}
		int prev_cross_id = 0;
		for (int i = 0; i < polygonvertices_count; i++) {
			Vector2 crosspoint = FindCrossPoint(polygonvertices[i], polygonvertices[(i + 1) % polygonvertices_count], prev_cross_id);
			addedvertices[polygonvertices_count + i] = crosspoint;
			crosspoints_prev_vertex_ids[i] = prev_cross_id;
		}
		/*Vector2 crosspoint = FindCrossPoint(polygonvertices[1], polygonvertices[0], prev_cross_id);
		addedvertices[polygonvertices_count * 2 - 1] = crosspoint;
		crosspoints_prev_vertex_ids[polygonvertices_count - 1] = prev_cross_id;*/
	}

	/*Returns is point under line v0->v1 (or point.x< line.x if line is vertical)*/
	bool NavMeshModifyer::IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse, bool strict) {
		bool vertical = false;
		float k = 0.0;
		float c = 0.0;
		if (v0.x == v1.x) {
			vertical = true;
		}
		else {
			k = (v0.y - v1.y) / (v0.x - v1.x);
			c = v0.y - k * v0.x;
		}
		if (vertical) {
			if (strict)
				return reverse ? point.x > v0.x : point.x < v0.x;
			else
				return reverse ? point.x >= v0.x : point.x <= v0.x;
		}
		if (strict) {
			if (reverse)
				return point.y > k*point.x + c;
			else
				return point.y < k*point.x + c;
		} else
			if (reverse)
				return point.y >= k*point.x + c;
			else
				return point.y <= k*point.x + c;
	}

	bool NavMeshModifyer::IsClockwise(FCArray<NavMeshVetrex> & polygon) {
		float sum = 0;
		for (int i = 0; i < polygon.size(); i++) {
			NavMeshVetrex v0 = polygon[i];
			NavMeshVetrex v1 = polygon[(i+1) % polygon.size()];
			sum += (v1.X-v0.X)*(v0.Y + v1.Y);
		}

		return sum > 0;
	}
}
