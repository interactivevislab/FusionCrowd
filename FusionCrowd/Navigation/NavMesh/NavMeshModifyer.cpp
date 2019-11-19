#include "NavMeshModifyer.h"


namespace FusionCrowd {
	NavMeshModifyer::NavMeshModifyer(NavMesh&  navmesh) : _navmesh(navmesh)
	{
	}


	NavMeshModifyer::~NavMeshModifyer()
	{
	}

	float NavMeshModifyer::CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon) {
		DirectX::SimpleMath::Vector2 *v0 = new DirectX::SimpleMath::Vector2(polygon[0].X, polygon[0].Y),
			*v1 = new DirectX::SimpleMath::Vector2(polygon[1].X, polygon[1].Y);
		return DivideNode(&_navmesh.nodes[0], v0, v1);
	}

	float NavMeshModifyer::DivideNode(NavMeshNode* node, DirectX::SimpleMath::Vector2* v0, DirectX::SimpleMath::Vector2* v1) {
		bool leave_up_node = true;
		//v0 should be left (or bottom) node
		if (v0->x > v1->x || (v0->x == v1->x && v0->y > v1->y)) {
			DirectX::SimpleMath::Vector2* tmp = v0;
			v0 = v1;
			v1 = tmp;
		}
		//calculate line coeff
		float k0 = 0.0;
		float c0 = 0.0;
		if (v0->x == v1->x) {
			//TODO
		}
		else {
			k0 = (v0->y - v1->y) / (v0->x - v1->x);
			c0 = v0->y - k0 * v0->x;
		}

		DirectX::SimpleMath::Vector2 cross_point1 = DirectX::SimpleMath::Vector2();
		cross_point1.x = 0.0;
		cross_point1.y = 0.0;
		size_t idpos_crosspoint00 = 0;
		size_t idpos_crosspoint01 = 0;
		bool point1_founded = false;
		DirectX::SimpleMath::Vector2 cross_point2 = DirectX::SimpleMath::Vector2();
		cross_point2.x = 0.0;
		cross_point2.y = 0.0;
		size_t idpos_crosspoint10 = 0;
		size_t idpos_crosspoint11 = 0;
		//find crossed edge
		NavMeshPoly* poly = node->getPolyRef();
		for (int i = 0; i < poly->vertCount; i++) {
			DirectX::SimpleMath::Vector2 tmp0 = DirectX::SimpleMath::Vector2();
			DirectX::SimpleMath::Vector2 tmp1 = DirectX::SimpleMath::Vector2();
			int tmp_id0 = i;
			int tmp_id1 = 0;
			if (i < poly->vertCount - 1) {
				tmp0.x = poly->vertices[poly->vertIDs[i]].x;
				tmp0.y = poly->vertices[poly->vertIDs[i]].y;
				tmp1.x = poly->vertices[poly->vertIDs[i + 1]].x;
				tmp1.y = poly->vertices[poly->vertIDs[i + 1]].y;
				tmp_id1 = i + 1;
			}
			else {
				tmp0.x = poly->vertices[poly->vertIDs[i]].x;
				tmp0.y = poly->vertices[poly->vertIDs[i]].y;
				tmp1.x = poly->vertices[poly->vertIDs[0]].x;
				tmp1.y = poly->vertices[poly->vertIDs[0]].y;
				tmp_id1 = 0;

			}
			//order poly verteces
			if (tmp0.x > tmp1.x) {
				DirectX::SimpleMath::Vector2 tmp = tmp0;
				//int tmpid = tmp_id0;
				tmp0 = tmp1;
				//tmp_id0 = tmp_id1;
				tmp1 = tmp;
				//tmp_id1 = tmpid;
			}
			if (tmp0.x == tmp1.x) {
				if (tmp0.y > tmp1.y) {
					DirectX::SimpleMath::Vector2 tmp = tmp0;
					//int tmpid = tmp_id0;
					tmp0 = tmp1;
					//tmp_id0 = tmp_id1;
					tmp1 = tmp;
					//tmp_id1 = tmpid;
				}
			}

			float k1 = 0.0;
			//line cross point
			float xev0 = 0.0;
			float yev0 = 0.0;

			if (tmp0.x == tmp1.x) {
				xev0 = tmp0.x;
				yev0 = k0 * xev0 + c0;
			}
			else {
				k1 = (tmp0.y - tmp1.y) / (tmp0.x - tmp1.x);
				if (k1 == k0) continue;
				float c1 = tmp0.y - k1 * tmp0.x;

				//line cross point
				xev0 = (c1 - c0) / (k0 - k1);
				yev0 = k1 * xev0 + c1;
			}
			//is on segment?
			float d = 0.0f;
			if (((xev0 + d >= tmp0.x && xev0 - d <= tmp1.x) || (xev0 - d <= tmp0.x && xev0 + d >= tmp1.x)) &&
				((yev0 + d >= tmp0.y && yev0 - d <= tmp1.y) || (yev0 - d <= tmp0.y && yev0 + d >= tmp1.y))) {
				if (!point1_founded) {
					cross_point1.x = xev0;
					cross_point1.y = yev0;
					point1_founded = true;
					idpos_crosspoint00 = tmp_id0;
					idpos_crosspoint01 = tmp_id1;
				}
				else {
					cross_point2.x = xev0;
					cross_point2.y = yev0;
					idpos_crosspoint10 = tmp_id0;
					idpos_crosspoint11 = tmp_id1;
				}
			}
		}
		//Create new vertices array
		DirectX::SimpleMath::Vector2* updvertices = new DirectX::SimpleMath::Vector2[_navmesh.vCount + 4];
		for (int i = 0; i < _navmesh.vCount; i++) {
			updvertices[i] = _navmesh.vertices[i];
		}
		delete[] _navmesh.vertices;
		_navmesh.vertices = updvertices;
		_navmesh.vCount = _navmesh.vCount + 4;

		//add new vertices
		_navmesh.vertices[_navmesh.vCount - 4] = cross_point1;
		_navmesh.vertices[_navmesh.vCount - 3] = cross_point2;
		_navmesh.vertices[_navmesh.vCount - 2] = *v0;
		_navmesh.vertices[_navmesh.vCount - 1] = *v1;

		//Calculate ids arrays size
		int creatednodeidscoutn = 0;
		int idstoleave = 0;
		for (int i = 0; i < poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[poly->vertIDs[i]];
			if ((vertex.y >= vertex.x*k0 + c0 && !leave_up_node) || (vertex.y < vertex.x*k0 + c0 && leave_up_node)) {
				idstoleave++;
			}
			else {
				creatednodeidscoutn++;
			}
		}
		creatednodeidscoutn += 4;

		//add node
		NavMeshNode updnode = NavMeshNode();
		updnode.setID(_navmesh.nCount - 1);
		updnode.setVertices(_navmesh.vertices);
		updnode._poly.vertCount = creatednodeidscoutn;
		updnode._poly.vertIDs = new unsigned int[creatednodeidscoutn];
		int addedids = 0;

		for (int i = 0; i < poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[poly->vertIDs[i]];
			if (!((vertex.y >= vertex.x*k0 + c0 && !leave_up_node) || (vertex.y < vertex.x*k0 + c0 && leave_up_node))) {
				updnode._poly.vertIDs[addedids] = poly->vertIDs[i];
				addedids++;
			}
			if (i == idpos_crosspoint00) {
				updnode._poly.vertIDs[addedids] = _navmesh.vCount - 4;
				addedids++;
				updnode._poly.vertIDs[addedids] = _navmesh.vCount - 1;
				addedids++;
				updnode._poly.vertIDs[addedids] = _navmesh.vCount - 2;
				addedids++;
				updnode._poly.vertIDs[addedids] = _navmesh.vCount - 3;
				addedids++;
			}
		}

		//transform leaved node
		unsigned int* updids = new unsigned int[idstoleave + 4];
		int idsdiff = 0;
		for (int i = 0; i < poly->vertCount; i++) {
			auto vertex = _navmesh.vertices[poly->vertIDs[i]];
			if ((vertex.y >= vertex.x*k0 + c0 && !leave_up_node) || (vertex.y < vertex.x*k0 + c0 && leave_up_node)) {
				updids[i + idsdiff] = poly->vertIDs[i];
			}
			else {
				idsdiff--;
			}
			if (i == idpos_crosspoint00) {
				idsdiff++;
				updids[i + idsdiff] = _navmesh.vCount - 3;
				idsdiff++;
				updids[i + idsdiff] = _navmesh.vCount - 2;
				idsdiff++;
				updids[i + idsdiff] = _navmesh.vCount - 1;
				idsdiff++;
				updids[i + idsdiff] = _navmesh.vCount - 4;
			}
		}
		delete[] poly->vertIDs;
		poly->vertIDs = updids;
		poly->vertCount = poly->vertCount + idsdiff;

		//update node array
		NavMeshNode* tmpNodes = new NavMeshNode[_navmesh.nCount + 1];
		if (_navmesh.nCount)
		{
			for (size_t i = 0; i < _navmesh.nCount; ++i)
			{
				tmpNodes[i] = _navmesh.nodes[i];
			}
			delete[] _navmesh.nodes;
		}
		_navmesh.nCount += 1;
		_navmesh.nodes = tmpNodes;
		_navmesh.nodes[_navmesh.nCount - 1] = updnode;

		return _navmesh.vCount;
	}
}
