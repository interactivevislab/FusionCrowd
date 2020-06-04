#include <fstream>
#include "NavMesh.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include <iostream>
#include <fstream>

//const std::string NavMesh::LABEL = "navmesh";

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMesh::NavMesh() : vCount(0), vertices(0x0),
                        nCount(0), nodes(0x0),
						eCount(0), edges(0x0),
						obstCount(0), obstacles(0x0),
                        nodeGroups()
	{
	}

#ifdef _WIN32
	// This disables a 64-bit compatibility warning - pushing a 64-bit value into a 32-bit value.
	//	In this case, I know the value in the pointers that are being re-interpreted as
	//  unsigned ints are REALLY just unsigned ints, so it is safe.
#pragma warning( disable : 4311 )
#endif
	bool NavMesh::finalize()
	{
		// All of the obst indices in the nodes need to be replaced with pointers
		// All of the obstacle indices in the nodes need to be replaced with pointers.
		for (size_t n = 0; n < nCount; ++n)
		{
			NavMeshNode& node = nodes[n];
			for (size_t e = 0; e < node._edgeCount; ++e)
			{
				// TODO: This might not work in building 64-bit code.
				//		The pointer will be larger than the unsigned int.  But as I'm pushing an
				//		unsigned int into a pointer slot, it'll probably be safe.  Needs to be
				//		tested.

				size_t eID = reinterpret_cast<size_t>(node._edges[e]);
				node._edges[e] = &edges[eID];
			}
			for (size_t o = 0; o < node._obstCount; ++o)
			{
				size_t oID = reinterpret_cast<size_t>(node._obstacles[o]);
				node._obstacles[o] = &obstacles[oID];
			}
			node._id = static_cast<unsigned int>(n);
			node._poly.vertices = vertices;
			node._poly.setBB();
		}

		// All of the node indices in the edges need to be replaced with pointers
		for (size_t e = 0; e < eCount; ++e)
		{
			NavMeshEdge& edge = edges[e];
			size_t nID = reinterpret_cast<size_t>(edge._node0);
			edge._node0 = &nodes[nID];

			nID = reinterpret_cast<size_t>(edge._node1);
			edge._node1 = &nodes[nID];
			// compute obst distance
			edge._distance = (edge._node0->getCenter() - edge._node1->getCenter()).Length();

			// Confirm that point is on the left when looking from node0
			if (FusionCrowd::Math::det(edge._dir, edge._node0->_center - edge._point) > 0.f)
			{
				NavMeshNode* tmp = edge._node0;
				edge._node0 = edge._node1;
				edge._node1 = tmp;
			}
		}

		std::vector<bool> processed(obstCount, false);
		for (size_t o = 0; o < obstCount; ++o)
		{
			obstacles[o]._id = o;
			if (processed[o]) continue;
			const size_t START = o;
			size_t curr = o;
			while (curr != NavMeshObstacle::NO_NEIGHBOR_OBST && !processed[curr])
			{
				processed[curr] = true;
				NavMeshObstacle& obst = obstacles[curr];
				size_t nID = reinterpret_cast<size_t>(obst._node);
				obst._node = &nodes[nID];

				nID = reinterpret_cast<size_t>(obst._nextObstacle);
				if (nID == NavMeshObstacle::NO_NEIGHBOR_OBST)
				{
					obst._nextObstacle = 0x0;
				}
				else
				{
					obst._nextObstacle = &obstacles[nID];
					//	Wire up "_prevObstacle" with the previous obstacle
					obstacles[nID]._prevObstacle = &obstacles[curr];
				}
				curr = nID;
			}

			// set open/closed
			if (curr == NavMeshObstacle::NO_NEIGHBOR_OBST || curr != START)
			{
				// set open
				Obstacle* obst = &obstacles[START];
				obst->setClosedState(false);
				while (obst->_nextObstacle != 0x0)
				{
					obst = obst->_nextObstacle;
					obst->setClosedState(false);
				}
			}
		}

		return true;
	}
#ifdef _WIN32
#pragma warning( default : 4311 )
#endif

	std::shared_ptr<NavMesh> NavMesh::Load(std::istream& f)
	{
		// TODO: Change this to support comments.

		// load vertices
		unsigned int vertCount;
		if (!(f >> vertCount))
		{
			return NULL;
		}

		std::shared_ptr<NavMesh> mesh = std::make_shared<NavMesh>();
		mesh->SetVertexCount(vertCount);
		float x, y;
		for (unsigned int v = 0; v < vertCount; ++v)
		{
			if (!(f >> x >> y))
			{
				return NULL;
			}
			mesh->SetVertex(v, x, y);
		}

		// load edges
		unsigned int edgeCount;
		if (!(f >> edgeCount))
		{
			return NULL;
		}
		mesh->SetEdgeCount(edgeCount);
		for (unsigned int e = 0; e < edgeCount; ++e)
		{
			NavMeshEdge& edge = mesh->GetEdge(e);
			if (!edge.loadFromAscii(f, mesh->vertices))
			{
				return NULL;
			}
		}

		// load obstacles
		unsigned int obstCount;
		if (!(f >> obstCount))
		{
			return NULL;
		}
		mesh->SetObstacleCount(obstCount);
		for (unsigned int o = 0; o < obstCount; ++o)
		{
			NavMeshObstacle& obst = mesh->GetObstacle(o);
			if (!obst.LoadFromAscii(f, mesh->vertices))
			{
				return NULL;
			}
		}

		unsigned int totalN = 0;
		unsigned int n = 0;
		// load node group
		while (!f.eof())
		{
			std::string grpName;
			if (!(f >> grpName))
			{
				if (f.eof())
				{
					break;
				}
				else
				{
					return 0x0;
				}
			}
			// load nodes
			unsigned int nCount;
			if (!(f >> nCount))
			{
				return 0x0;
			}

			totalN += nCount;
			mesh->AddGroup(grpName, nCount);
			assert(totalN == mesh->getNodeCount() &&
				"Data management problem -- tracked node count does not match "
				"in-memory node count");

			for (; n < totalN; ++n)
			{
				NavMeshNode& node = mesh->GetNodeByPos(n);
				if (!node.loadFromAscii(f))
				{
					return 0x0;
				}

				node.setID(n);
				node.setVertices(mesh->GetVertices());
			}
		}

		if (!mesh->finalize())
		{
			return NULL;
		}

		mesh->CheckObstaclesDirection();

		return mesh;
	}

	void NavMesh::CheckObstaclesDirection()
	{
		std::vector<bool> processed(obstacles.size(), false);
		for (size_t o = 0; o < obstacles.size(); ++o)
		{
			if (processed[o]) continue;

			NavMeshObstacle & obst = obstacles[o];
			if(obst._nextObstacle == nullptr || obst._prevObstacle == nullptr) continue;

			// This is probably a cycle.
			size_t reverted = 0;
			size_t length = 0;

			obst = (NavMeshObstacle &) *obst._prevObstacle;
			size_t finish = obst.getId();
			do
			{
				obst = (NavMeshObstacle &) *obst._nextObstacle;

				if(Math::leftOf(obst._prevObstacle->getP0(), obst.getP0(), obst.getP1()) >= 0)
					reverted++;


				length++;
			}
			while(obst.getId() != finish && obst._nextObstacle != nullptr);

			if(obst.getId() == finish && reverted == 0 && length > 0)
			{
				// This is a cycle and it is a convex needs to be reversed
				ReverseCycle(obst.getId());
			}
		}
	}

	void NavMesh::ReverseCycle(size_t obstId)
	{
		// We assume that cycle exists

		NavMeshObstacle & start = obstacles[obstId];
		NavMeshObstacle & cur = start;
		do
		{
			auto tmp = cur._nextObstacle;
			cur._nextObstacle = cur._prevObstacle;
			cur._prevObstacle = tmp;
			cur = (NavMeshObstacle &) *cur._prevObstacle;

		} while(cur.getId() != start.getId());
	}

#pragma region  Vertex
	void NavMesh::SetVertexCount(size_t count)
	{
		if (vCount)
		{
			delete[] vertices;
		}
		vCount = count;
		vertices = new Vector2[vCount];
	}

	void NavMesh::SetVertex(unsigned int i, float x, float y)
	{
		vertices[i] = Vector2(x, y);
	}
#pragma endregion

#pragma region  Edge
	void NavMesh::SetEdgeCount(size_t count)
	{
		if (eCount)
		{
			delete[] edges;
		}
		eCount = count;
		edges = new NavMeshEdge[eCount];
	}

	NavMeshEdge& NavMesh::GetEdge(unsigned int i)
	{
		return edges[i];
	}
#pragma endregion

#pragma region Obstacle
	void NavMesh::SetObstacleCount(size_t count)
	{
		obstCount = count;
		obstacles.reserve(obstCount);
		for(size_t i = 0; i < count; i++)
			obstacles.push_back(NavMeshObstacle());
	}

	NavMeshObstacle& NavMesh::GetObstacle(unsigned int i)
	{
		return obstacles[i];
	}
#pragma endregion

#pragma region Node
	bool NavMesh::AddGroup(const std::string& grpName, size_t grpSize)
	{
		if (nodeGroups.find(grpName) != nodeGroups.end())
		{
			return false;
		}
		size_t first = nCount;
		size_t last = first + grpSize - 1;
		nodeGroups[grpName] = NMNodeGroup(static_cast<unsigned int>(first),
		                                  static_cast<unsigned int>(last));

		// Now extend the node memory
		NavMeshNode* tmpNodes = new NavMeshNode[nCount + grpSize];
		if (nCount)
		{
			for (size_t i = 0; i < nCount; ++i)
			{
				tmpNodes[i] = nodes[i];
			}
			delete[] nodes;
		}
		nCount += grpSize;
		nodes = tmpNodes;
		return true;
	}

	NavMeshNode& NavMesh::GetNodeByPos(unsigned int id)
	{
		return nodes[id];
	}


	NavMeshNode* NavMesh::GetNodeByID(unsigned int id) {
		if(id >= nCount)
			return nullptr;

		return &nodes[id];
	}

	const NMNodeGroup* NavMesh::getNodeGroup(const std::string& grpName) const
	{
		std::map<const std::string, NMNodeGroup>::const_iterator itr = nodeGroups.find(grpName);
		NMNodeGroup* grp = 0x0;
		if (itr != nodeGroups.end())
		{
			return &(itr->second);
		}
		return grp;
	}

#pragma endregion

	void NavMesh::clear()
	{
		if (vCount)
		{
			vCount = 0;
			delete[] vertices;
			vertices = 0x0;
		}

		if (nCount)
		{
			nCount = 0;
			delete[] nodes;
			nodes = 0x0;
		}

		if (eCount)
		{
			eCount = 0;
			delete[] edges;
			edges = 0x0;
		}
	}

	NavMesh::~NavMesh()
	{
		clear();
	}

	size_t NavMesh::GetVersion() const
	{
		return _version;
	}

	void NavMesh::IncVersion()
	{
		_version++;
	}

	size_t NavMesh::GetVertexCount() {
		return vCount;
	}

	bool NavMesh::GetVertices(FCArray<NavMeshVetrex> & output) {

		if (output.size() < vCount)
		{
			return false;
		}

		int i = 0;
		for (int i = 0; i < vCount; i++)
		{
			NavMeshVetrex vertex = NavMeshVetrex();
			vertex.X = vertices[i].x;
			vertex.Y = vertices[i].y;
			output[i] = vertex;
		}
		return true;
	}


	size_t NavMesh::GetNodesCount() {
		int res = 0;
		for (int i = 0; i < nCount; i++) {
			if (!nodes[i].deleted) res++;
		}
		return res;
	}
	size_t NavMesh::GetNodeVertexCount(size_t node_id) {
		size_t not_deleted_id = 0;
		size_t true_nid = 0;
		for (int i = 0; i < nCount; i++) {
			if (!nodes[i].deleted) {
				if (not_deleted_id == node_id) {
					true_nid = i;
					break;
				}
				not_deleted_id++;
			}
		}
		return nodes[true_nid]._poly.vertCount;
	}

	bool NavMesh::GetNodeVertexInfo(FCArray<int> & output, size_t node_id) {
		size_t not_deleted_id = 0;
		size_t true_nid = 0;
		for (int i = 0; i < nCount; i++) {
			if (!nodes[i].deleted) {
				if (not_deleted_id == node_id) {
					true_nid = i;
					break;
				}
				not_deleted_id++;
			}
		}
		for (int i = 0; i < nodes[true_nid]._poly.vertCount; i++) {
			output[i] = nodes[true_nid]._poly.vertIDs[i];
			//if (nodes[node_id]._poly.vertIDs[i] < 0) return false;
		}
		return true;
	}

	size_t NavMesh::GetEdgesCount() {
		return eCount;
	}

	bool NavMesh::GetEdges(FCArray<EdgeInfo> & output) {
		if (output.size() < eCount)
		{
			return false;
		}

		for (int i = 0; i < eCount; i++)
		{
			output[i].error_code = 0;
			output[i] = EdgeInfo();
			output[i].x1 = edges[i].getP0().x;
			output[i].y1 = edges[i].getP0().y;
			output[i].x2 = edges[i].getP1().x;
			output[i].y2 = edges[i].getP1().y;
			output[i].nx0 = edges[i].getFirstNode()->getCenter().x;
			output[i].ny0 = edges[i].getFirstNode()->getCenter().y;
			output[i].nx1 = edges[i].getSecondNode()->getCenter().x;
			output[i].ny1 = edges[i].getSecondNode()->getCenter().y;
			if (edges[i].getFirstNode()->deleted) output[i].error_code += 1;
			if (edges[i].getSecondNode()->deleted) output[i].error_code += 1;
		}
		return true;
	}

	size_t NavMesh::GetObstaclesCount() {
		return obstCount;
	}

	bool NavMesh::GetObstacles(FCArray<EdgeInfo> & output)
	{
		if (output.size() < obstCount)
		{
			return false;
		}
		for (int i = 0; i < obstCount; i++)
		{
			output[i].error_code = 0;
			output[i] = EdgeInfo();
			output[i].x1 = obstacles[i].getP0().x;
			output[i].y1 = obstacles[i].getP0().y;
			output[i].x2 = obstacles[i].getP1().x;
			output[i].y2 = obstacles[i].getP1().y;
			auto node = obstacles[i].getNode();
			if (node->deleted) output[i].error_code = 1;
			output[i].nx0 = node->_center.x;
			output[i].ny0 = node->_center.y;
		}
		return true;
	}

	bool NavMesh::ExportNavMeshToFile(char* file_name)
	{
		std::ofstream file;
		file.open(std::string(file_name));
		std::vector<Vector2> tmpv;
		//vertices
		file << vCount + eCount*2 +obstCount*2 << '\n';
		for (int i = 0; i < vCount; i++) {
			file << vertices[i].x << ' ' << vertices[i].y << '\n';
			tmpv.push_back(vertices[i]);
		}
		for (int i = 0; i < eCount; i++) {
			auto& edge = edges[i];
			file << edge.getP0().x << ' ' << edge.getP0().y << '\n';
			file << edge.getP1().x << ' ' << edge.getP1().y << '\n';
			tmpv.push_back(Vector2(edge.getP0().x, edge.getP0().y));
			tmpv.push_back(Vector2(edge.getP1().x, edge.getP1().y));
			if ((edge.getP0() - edge.getP1()).Length() < 0.01) throw 1;
		}
		for (int i = 0; i < obstCount; i++) {
			auto& obst = obstacles[i];
			file << obst.getP0().x << ' ' << obst.getP0().y << '\n';
			file << obst.getP1().x << ' ' << obst.getP1().y << '\n';
			tmpv.push_back(Vector2(obst.getP0().x, obst.getP0().y));
			tmpv.push_back(Vector2(obst.getP1().x, obst.getP1().y));
		}

		//create true node id map
		std::map<size_t, size_t> node_id_map = std::map<size_t, size_t>();
		size_t true_node_count = 0;
		for (int i = 0; i < nCount; i++) {
			if (!nodes[i].deleted) {
				node_id_map.insert({i, true_node_count});
				true_node_count++;
			}
		}

		//edges
		std::map<NavMeshEdge*, size_t> edge_id_map = std::map<NavMeshEdge*, size_t>();
		file << eCount << '\n';
		for (int i = 0; i < eCount; i++) {
			auto& edge = edges[i];
			edge_id_map.insert({ &edges[i], i });
			size_t n0 = node_id_map.at(edge.getFirstNode()->_id);
			size_t n1 = node_id_map.at(edge.getSecondNode()->_id);
			size_t v0 = vCount + i * 2;
			size_t v1 = v0 + 1;
			if (tmpv[v0] == tmpv[v1]) throw 1;
			file << v0 << ' ' << v1 << ' ' << n0 << ' ' << n1 << '\n';
		}

		//obstacles
		std::map<NavMeshObstacle*, size_t> obst_id_map = std::map<NavMeshObstacle*, size_t>();
		file << obstCount << '\n';
		for (int i = 0; i < obstCount; i++) {
			auto& obst = obstacles[i];
			obst_id_map.insert({ &obstacles[i], i });
			size_t n0 = node_id_map.at(obst.getNode()->_id);
			size_t v0 = vCount + eCount * 2 + i * 2;
			size_t v1 = v0 + 1;
			if (tmpv[v0] == tmpv[v1]) throw 1;
			file << v0 << ' ' << v1 << ' ' << n0 << ' ' << -1 << '\n';
		}

		//nodes
		file << "nodes\n";
		file << true_node_count << '\n';


		for (int i = 0; i < nCount; i++) {
			auto& node = nodes[i];
			if (node.deleted) continue;
			//center
			file << node._center.x << ' ' << node._center.y << '\n';
			//vertices
			file << node._poly.vertCount;
			for (int j = 0; j < node._poly.vertCount; j++) {
				file << ' ' << node._poly.vertIDs[j] ;
			}
			file << '\n';
			//offset
			file << 0.0f << ' ' << 0.0f << ' ' << 0.0f << '\n';

			//edges
			file << node._edgeCount;
			for (int j = 0; j < node._edgeCount; j++) {
				file << ' ' << edge_id_map.at(node._edges[j]);
			}
			file << " \n";

			//obstacles
			file << node._obstCount;
			for (int j = 0; j < node._obstCount; j++) {
				file << ' ' << obst_id_map.at(node._obstacles[j]);
			}
			file << " \n";
		}

		file.close();
		return true;
	}
}
