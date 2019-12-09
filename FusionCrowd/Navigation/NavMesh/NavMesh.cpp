#include <fstream>
#include "NavMesh.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h";

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
		// All of the edge indices in the nodes need to be replaced with pointers
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
			node._poly.setBB(vertices);
		}

		// All of the node indices in the edges need to be replaced with pointers
		for (size_t e = 0; e < eCount; ++e)
		{
			NavMeshEdge& edge = edges[e];
			size_t nID = reinterpret_cast<size_t>(edge._node0);
			edge._node0 = &nodes[nID];

			nID = reinterpret_cast<size_t>(edge._node1);
			edge._node1 = &nodes[nID];
			// compute edge distance
			edge._distance = (edge._node0->getCenter() - edge._node1->getCenter()).Length();

			// Confirm that point is on the left when looking from node0
			if (FusionCrowd::MathUtil::det(edge._dir, edge._node0->_center - edge._point) > 0.f)
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
				NavMeshNode& node = mesh->GetNode(n);
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

				if(MathUtil::leftOf(obst._prevObstacle->getP0(), obst.getP0(), obst.getP1()) >= 0)
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

	NavMeshNode& NavMesh::GetNode(unsigned int i)
	{
		return nodes[i];
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
		return nCount;
	}
	size_t NavMesh::GetNodeVertexCount(size_t node_id) {
		return nodes[node_id]._poly.vertCount;
	}

	bool NavMesh::GetNodeVertexInfo(FCArray<int> & output, size_t node_id) {
		for (int i = 0; i < nodes[node_id]._poly.vertCount; i++) {
			output[i] = nodes[node_id]._poly.vertIDs[i];
			//if (nodes[node_id]._poly.vertIDs[i] < 0) return false;
		}
		return true;
	}
}
