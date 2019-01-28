#include <fstream>
#include "NavMesh.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h";

NavMesh::NavMesh()
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
	for (size_t n = 0; n < nCount; ++n) {
		NavMeshNode & node = nodes[n];
		for (size_t e = 0; e < node._edgeCount; ++e) {
			// TODO: This might not work in building 64-bit code.
			//		The pointer will be larger than the unsigned int.  But as I'm pushing an
			//		unsigned int into a pointer slot, it'll probably be safe.  Needs to be
			//		tested.

			size_t eID = reinterpret_cast<size_t>(node._edges[e]);
			node._edges[e] = &edges[eID];
		}
		for (size_t o = 0; o < node._obstCount; ++o) {
			size_t oID = reinterpret_cast<size_t>(node._obstacles[o]);
			node._obstacles[o] = &obstacles[oID];
		}
		node._id = static_cast<unsigned int>(n);
		node._poly.setBB(vertices);
	}

	// All of the node indices in the edges need to be replaced with pointers
	for (size_t e = 0; e < eCount; ++e) {
		NavMeshEdge & edge = edges[e];
		size_t nID = reinterpret_cast<size_t>(edge._node0);
		edge._node0 = &nodes[nID];

		nID = reinterpret_cast<size_t>(edge._node1);
		edge._node1 = &nodes[nID];
		// compute edge distance
		edge._distance = abs(edge._node0->getCenter() - edge._node1->getCenter());

		// Confirm that point is on the left when looking from node0
		if (det(edge._dir, edge._node0->_center - edge._point) > 0.f) {
			NavMeshNode * tmp = edge._node0;
			edge._node0 = edge._node1;
			edge._node1 = tmp;
		}
	}

	std::vector< bool > processed(obstCount, false);
	for (size_t o = 0; o < obstCount; ++o) {
		obstacles[o]._id = o;
		if (processed[o]) continue;
		const size_t START = o;
		size_t curr = o;
		while (curr != NavMeshObstacle::NO_NEIGHBOR_OBST && !processed[curr]) {
			processed[curr] = true;
			NavMeshObstacle & obst = obstacles[curr];
			size_t nID = reinterpret_cast<size_t>(obst._node);
			obst._node = &nodes[nID];

			nID = reinterpret_cast<size_t>(obst._nextObstacle);
			if (nID == NavMeshObstacle::NO_NEIGHBOR_OBST) {
				obst._nextObstacle = 0x0;
			}
			else {
				obst._nextObstacle = &obstacles[nID];
				//	Wire up "_prevObstacle" with the previous obstacle
				obstacles[nID]._prevObstacle = &obstacles[curr];
			}
			curr = nID;
		}
		// set open/closed
		if (curr == NavMeshObstacle::NO_NEIGHBOR_OBST ||
			curr != START) {	// set open
			Obstacle * obst = &obstacles[START];
			obst->setClosedState(false);
			while (obst->_nextObstacle != 0x0) {
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

bool NavMesh::Load(const std::string FileName)
{
	std::ifstream f;
	f.open(fileName.c_str(), std::ios::in);

	if (!f.is_open()) {
		return false;
	}

	// load vertices
	unsigned int vertCount;
	if (!(f >> vertCount)) {
		return false;
	}
	SetVertexCount(vertCount);
	float x, y;
	for (unsigned int v = 0; v < vertCount; ++v) {
		if (!(f >> x >> y)) {
			Destroy();
			return false;
		}
		SetVertex(v, x, y);
	}
	// load edges
	unsigned int edgeCount;
	if (!(f >> edgeCount)) {
		Destroy();
		return false;
	}
	SetEdgeCount(edgeCount);
	for (unsigned int e = 0; e < edgeCount; ++e) {
		NavMeshEdge & edge = GetEdge(e);
		if (!edge.loadFromAscii(f, vertices)) {
			Destroy();
			return false;
		}
	}
	// load obstacles
	unsigned int obstCount;
	if (!(f >> obstCount)) {
		Destroy();
		return false;
	}
	SetObstacleCount(obstCount);
	for (unsigned int o = 0; o < obstCount; ++o) {
		NavMeshObstacle & obst = GetObstacle(o);
		if (!obst.LoadFromAscii(f, vertices)) {
			Destroy();
			return false;
		}
	}
	unsigned int totalN = 0;
	unsigned int n = 0;
	// load node group
	while (!f.eof()) {
		std::string grpName;
		if (!(f >> grpName)) {
			if (f.eof()) {
				break;
			}
			else {
				Destroy();
				return 0x0;
			}
		}
		// load nodes
		unsigned int nCount;
		if (!(f >> nCount)) {
			Destroy();
			return false;
		}

		totalN += nCount;
		AddGroup(grpName, nCount);

		for (; n < totalN; ++n) {
			NavMeshNode & node = GetNode(n);
			if (!node.loadFromAscii(f)) {
				Destroy();
				return false;
			}

			node.setID(n);
			node.setVertices(GetVertices());
		}
	}

	if (finalize()) {
		Destroy();
		return 0x0;
	}
	return true;
}

#pragma region  Vertex
void NavMesh::SetVertexCount(size_t count)
{
	if (vCount)
	{
		delete[] vertices;
	}
	vCount = count;
	vertices = new FusionCrowd::Math::Vector2[vCount];
}

void NavMesh::SetVertex(unsigned int i, float x, float y)
{
	vertices[i].set(x, y);
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

NavMeshEdge & NavMesh::GetEdge(unsigned int i)
{
	return edges[i];
}
#pragma endregion

#pragma region Obstacle
void NavMesh::SetObstacleCount(size_t count)
{
	if (obstCount)
	{
		delete[] obstacles;
	}
	obstCount = count;
	obstacles = new NavMeshObstacle[obstCount];
}

NavMeshObstacle & NavMesh::GetObstacle(unsigned int i)
{
	return obstacles[i];
}
#pragma endregion

#pragma region Node
bool NavMesh::AddGroup(const std::string & grpName, size_t grpSize)
{
	if (nodeGroups.find(grpName) != nodeGroups.end()) {
		return false;
	}
	size_t first = nCount;
	size_t last = first + grpSize - 1;
	nodeGroups[grpName] = NMNodeGroup(static_cast<unsigned int>(first),
		static_cast<unsigned int>(last));

	// Now extend the node memory
	NavMeshNode * tmpNodes = new NavMeshNode[nCount + grpSize];
	if (nCount) {
		for (size_t i = 0; i < nCount; ++i) {
			tmpNodes[i] = nodes[i];
		}
		delete[] nodes;
	}
	nCount += grpSize;
	nodes = tmpNodes;
	return true;
}

NavMeshNode & NavMesh::GetNode(unsigned int i)
{
	return nodes[i];
}
#pragma endregion

NavMesh::~NavMesh()
{
}
