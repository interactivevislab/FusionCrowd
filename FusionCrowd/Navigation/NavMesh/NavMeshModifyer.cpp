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
		Initialize(polygon);
		/*Vector2 *v0 = new Vector2(polygon[0].X, polygon[0].Y),
			*v1 = new Vector2(polygon[1].X, polygon[1].Y);
		return DivideNode(&_navmesh.nodes[0], v0, v1);*/
		_current_poly = &_navmesh.nodes[0]._poly;
		FillAddedVertices();
		int res = AddNodes();
		ModifyHomeNode();
		Finalize();
		return res;
	}

	void NavMeshModifyer::ModifyHomeNode() {
		//calculate vertices num
		int nodes_to_stay = 0;
		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			if (IsPointUnderLine(polygonvertices[0], polygonvertices[1], vertex, !_leave_up_node)){
				nodes_to_stay++;
			}
		}
		nodes_to_stay += 4;

		//transform leaved node
		unsigned int* updids = new unsigned int[nodes_to_stay];
		int idsdiff = 0;
		for (int i = 0; i < _current_poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
			bool vertexadded = false;
			if (IsPointUnderLine(polygonvertices[0], polygonvertices[1], vertex, !_leave_up_node)) {
				updids[i + idsdiff] = _current_poly->vertIDs[i];
				vertexadded = true;
			} else {
				idsdiff--;
			}
			if (i == crosspoints_prev_vertex_ids[polygonvertices_count - 1]) {
				if (vertexadded) {
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + addedvertices_count - 1;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + 1;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + polygonvertices_count;
				} else {
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + polygonvertices_count;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + 1;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount;
					idsdiff++;
					updids[i + idsdiff] = _navmesh.vCount + addedvertices_count - 1;
				}
			}
		}
		delete[] _current_poly->vertIDs;
		_current_poly->vertIDs = updids;
		_current_poly->vertCount = _current_poly->vertCount + idsdiff;
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
	void NavMeshModifyer::Initialize(FCArray<NavMeshVetrex> & polygon) {
		polygonvertices_count = polygon.size();
		polygonvertices = new Vector2[polygonvertices_count];
		for (int i = 0; i < polygonvertices_count; i++) {
			DirectX::SimpleMath::Vector2 v = DirectX::SimpleMath::Vector2(polygon[i].X, polygon[i].Y);
			polygonvertices[i] = v;
		}

		addednodes_count = polygonvertices_count - 1;;
		addednodes = new NavMeshNode[addednodes_count];

		addedvertices_count = polygonvertices_count*2;
		addedvertices = new DirectX::SimpleMath::Vector2[addedvertices_count];

		crosspoints_prev_vertex_ids_num = polygonvertices_count;
		crosspoints_prev_vertex_ids = new int[polygonvertices_count];
	}

	/*Adds additional nodes*/
	int NavMeshModifyer::AddNodes() {
		//which side should be leave?
		_leave_up_node = !IsPointUnderLine(polygonvertices[0], polygonvertices[1], polygonvertices[2]);

		for (int j = 1; j < polygonvertices_count; j++) {
			NavMeshNode updnode = NavMeshNode();
			updnode.setID(_navmesh.nCount + j - 1);
			int vert_count = 0;
			bool node_side0 = !IsPointUnderLine(polygonvertices[j - 1], polygonvertices[j],
				addedvertices[polygonvertices_count + j]);
			bool node_side1 = !IsPointUnderLine(polygonvertices[j], polygonvertices[(j + 1) % polygonvertices_count],
				addedvertices[polygonvertices_count + j - 1]);
			for (int i = 0; i < _current_poly->vertCount; i++) {
				auto vertex = _navmesh.vertices[_current_poly->vertIDs[i]];
				if (IsPointUnderLine(polygonvertices[j-1], polygonvertices[j], vertex, node_side0, true) &&
					IsPointUnderLine(polygonvertices[j], polygonvertices[(j + 1) % polygonvertices_count], vertex, node_side1)
					&& IsPointUnderLine(polygonvertices[0], polygonvertices[1], vertex, _leave_up_node, true)) {
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
				if (IsPointUnderLine(polygonvertices[j - 1], polygonvertices[j], vertex, node_side0, true) &&
					IsPointUnderLine(polygonvertices[j], polygonvertices[(j + 1) % polygonvertices_count], vertex, node_side1) &&
					IsPointUnderLine(polygonvertices[0], polygonvertices[1], vertex, _leave_up_node, true)) {
					updnode._poly.vertIDs[addedids] = _current_poly->vertIDs[i];
					addedids++;
					vertex_added = true;
				}
				if (i == crosspoints_prev_vertex_ids[j  - 1]) {
					if (vertex_added) {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j - 1; //cross point
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + j; //polygon node
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count); //polygon node
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j; //cross point
						addedids++;
					}
					else {
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j - 1;
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + j;
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + ((j + 1) % polygonvertices_count);
						addedids++;
						updnode._poly.vertIDs[addedids] = _navmesh.vCount + polygonvertices_count + j;
						addedids++;
					}
				}
			}

			addednodes[j-1] = updnode;
		}
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
					(v1.x > v0.x && xcross > v1.x) ||
					(v1.x < v0.x && xcross < v1.x) ||
					(v1.x == v0.x && v1.y> v0.y && ycross > v1.y) ||
					(v1.x == v0.x && v1.y < v0.y && ycross < v1.y)
					){
					out_prev_cross_id = i;
					return Vector2(xcross, ycross);
				}
				else continue;
			}
		}
		return Vector2(0, 0);
	}

	void NavMeshModifyer::FillAddedVertices() {
		for (int i = 0; i < polygonvertices_count; i++) {
			addedvertices[i] = polygonvertices[i];
		}
		int prev_cross_id = 0;
		for (int i = 0; i < polygonvertices_count - 1; i++) {
			Vector2 crosspoint = FindCrossPoint(polygonvertices[i], polygonvertices[i + 1], prev_cross_id);
			addedvertices[polygonvertices_count + i] = crosspoint;
			crosspoints_prev_vertex_ids[i] = prev_cross_id;
		}
		Vector2 crosspoint = FindCrossPoint(polygonvertices[1], polygonvertices[0], prev_cross_id);
		addedvertices[polygonvertices_count * 2 - 1] = crosspoint;
		crosspoints_prev_vertex_ids[polygonvertices_count - 1] = prev_cross_id;
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
}
