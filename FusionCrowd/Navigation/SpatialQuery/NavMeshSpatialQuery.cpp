#include "NavMeshSpatialQuery.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class VisibilityCone
	{
	public:
		VisibilityCone(const Vector2 & dir0, const Vector2 & dir1)
		{
			if (MathUtil::det(dir0, dir1) > 0) {
				_right = dir0;
				_left = dir1;
			}
			else {
				_right = dir1;
				_left = dir0;
			}
		}

		bool IsVisible(const Vector2 & p0, const Vector2 & p1) const
		{
			// if either point is visible, then it is true
			float right0 = MathUtil::det(p0, _right);
			float left0 = MathUtil::det(_left, p0);
			if (right0 <= 0 && left0 <= 0) return true;

			float right1 = MathUtil::det(p1, _right);
			float left1 = MathUtil::det(_left, p1);
			if (right1 <= 0 && left1 <= 0) return true;

			// otherwise, if the two points lie outside the cone on opposite sides
			//		then the cone intersects the center.
			if (right0 > 0 && left1 > 0) {
				return MathUtil::det(p1 - p0, -p0) > 0.f;
			}
			else if (right1 > 0 && left0 > 0) {
				return MathUtil::det(p0 - p1, -p1) > 0.f;
			}
			return false;
		}

		bool IsVisible(const Vector2 & p) const
		{
			if (MathUtil::det(p, _right) > 0) return false;
			if (MathUtil::det(_left, p) > 0) return false;
			return true;
		}

		bool Intersect(const VisibilityCone & cone)
		{
			Vector2 iRight(MathUtil::det(_right, cone._right) > 0 ? cone._right : _right);
			Vector2 iLeft(MathUtil::det(_left, cone._left) > 0 ? _left : cone._left);
			if (MathUtil::det(iRight, iLeft) > 0) {
				_right = iRight;
				_left = iLeft;
				return true;
			}
			else {
				return false;
			}
		}

		Vector2 _left;
		Vector2 _right;
	};

	class NeighborEntry
	{
	public:
		NeighborEntry(float distSq, const VisibilityCone & cone, unsigned int nodeID) :
			_distSq(distSq), _cone(cone), _nodeID(nodeID)
		{
		}
		bool operator<(const NeighborEntry & entry) const
		{
			return _distSq < entry._distSq;
		}

		float _distSq;
		VisibilityCone _cone;
		unsigned int _nodeID;
	};

	NavMeshSpatialQuery::NavMeshSpatialQuery(): _localizer(NULL)
	{
	}

	void NavMeshSpatialQuery::SetAgents(const std::vector<Agent * > & agents)
	{
		_agents.insert(_agents.begin(), agents.begin(), agents.end());
	}

	void NavMeshSpatialQuery::AgentQuery(ProximityQuery *filter) const
	{
		float range = filter->GetMaxAgentRange();
		AgentQuery(filter, range);
	}

	void NavMeshSpatialQuery::AgentQuery(ProximityQuery *filter, float &rangeSq) const
	{
		Vector2 pt = filter->GetQueryPoint();
		unsigned int currNode = _localizer->getNode(pt);
		assert(currNode != NavMeshLocation::NO_NODE &&
			"Can't use NavMesh for spatial query if the point isn't on the mesh");

		// This does not need any synchronization elements
		//	The writing and the reading happen in two, independent computational
		//	stages.  (i.e., the writing to the node occupancy happens in a task.)
		//	This is all read-only operations and can be done simultaneously.
		const OccupantSet * occupants = _localizer->getNodeOccupants(currNode);
		if (occupants->size() > 1) {
			OccupantSetCItr itr = occupants->begin();
			for (; itr != occupants->end(); ++itr) {

				const Agent * candidate = _agents[*itr];
				float distSq = (candidate->pos - pt).LengthSquared();
				if (distSq <= rangeSq) {
					// NOTE: This call might change rangeSq; it may shrink based on the most
					// distant neighbor
					filter->FilterAgent(candidate, distSq);

					rangeSq = filter->GetMaxAgentRange();
				}


			}
		}

		NavMeshPtr navMesh = _localizer->getNavMesh();
		// Track which nodes have been visited
		std::set< unsigned int > visited;
		visited.insert((unsigned int)currNode);
		// now create a min heap of nearby navigation mesh nodes to explore for neighbor
		// candidates
		std::list< NeighborEntry > queue;

		// seed the queue with this node's adjacent nodes
		const NavMeshNode & node = navMesh->GetNode((unsigned int)currNode);
		const size_t EDGE_COUNT = node.getEdgeCount();
		for (size_t e = 0; e < EDGE_COUNT; ++e) {
			const NavMeshEdge * edge = node.getEdge(e);
			const NavMeshNode * otherNode = edge->getOtherByID(currNode);
			visited.insert(otherNode->getID());
			float distSq = edge->getSqDist(pt);
			if (distSq <= rangeSq) {
				queue.push_back(NeighborEntry(distSq, VisibilityCone(edge->getP0() - pt,
					edge->getP1() - pt), otherNode->getID()));
				//cones.push_back( VisibilityCone( edge->getP0() - P, edge->getP1() - P ) );
				// edge is close enough that portions of the node are reachable
				//queue.push_back( NeighborEntry( distSq, otherNode->getID() ) );
			}
		}

		while (queue.size() > 0) {
			NeighborEntry nbrEntry = *(queue.begin());
			queue.pop_front();
			if (nbrEntry._distSq > rangeSq) continue;

			const OccupantSet * occupants = _localizer->getNodeOccupants(nbrEntry._nodeID);
			if (occupants->size() > 0) {
				OccupantSetCItr itr = occupants->begin();
				for (; itr != occupants->end(); ++itr) {
					const Agent * candidate = _agents[*itr];
					Vector2 disp(candidate->pos - pt);
					float distSq = disp.LengthSquared();
					if (distSq <= rangeSq) {
						if (nbrEntry._cone.IsVisible(disp)) {
							filter->FilterAgent(candidate, distSq);
						}

						rangeSq = filter->GetMaxAgentRange();
					}
				}
			}

			const NavMeshNode & node = navMesh->GetNode(nbrEntry._nodeID);
			const size_t EDGE_COUNT = node.getEdgeCount();
			for (size_t e = 0; e < EDGE_COUNT; ++e) {
				const NavMeshEdge * edge = node.getEdge(e);

				const NavMeshNode * otherNode = edge->getOtherByID(nbrEntry._nodeID);
				if (visited.find(otherNode->getID()) != visited.end()) continue;
				visited.insert(otherNode->getID());

				float distSq = edge->getSqDist(pt);
				if (distSq <= rangeSq) {
					Vector2 disp1 = edge->getP0() - pt;
					Vector2 disp2 = edge->getP1() - pt;
					VisibilityCone cone(disp1, disp2);
					if (cone.Intersect(nbrEntry._cone)) {
						queue.push_back(NeighborEntry(distSq, cone, otherNode->getID()));
					}
				}
			}
		}
	}

	bool NavMeshSpatialQuery::QueryVisibility(const Vector2& q1, const Vector2& q2, float radius) const
	{
		return true;
	}

	void NavMeshSpatialQuery::ProcessObstacles()
	{
		// Compute obstacle convexity -- this assumes all closed polygons
		NavMeshPtr navMesh = _localizer->getNavMesh();
		const unsigned int OBST_COUNT = static_cast<unsigned int>(navMesh->getObstacleCount());
		for (unsigned int o = 0; o < OBST_COUNT; ++o) {
			NavMeshObstacle & obst = navMesh->GetObstacle(o);
			if (obst._prevObstacle) {
				obst._isConvex = MathUtil::leftOf(obst._prevObstacle->getP0(),
					obst.getP0(), obst.getP1()) >= 0;
			}
			else {
				obst._isConvex = true;
			}
		}
	}

	void NavMeshSpatialQuery::ObstacleQuery(ProximityQuery *filter) const
	{
		float range = filter->GetMaxObstacleRange();
		ObstacleQuery(filter, range);
	}

	void NavMeshSpatialQuery::ObstacleQuery(ProximityQuery *filter, float rangeSq) const
	{
		Vector2 pt = filter->GetQueryPoint();

		Agent * agent = dynamic_cast<Agent*>(filter);
		size_t currNode = NavMeshLocation::NO_NODE;
		if (agent != 0x0) {
			currNode = _localizer->getNode(agent->pos);
		}
		else {
			size_t currNode = _localizer->getNode(pt);
		}

		assert(currNode != NavMeshLocation::NO_NODE &&
			"Can't use NavMesh for spatial query if the point isn't on the mesh");

		const NavMeshPtr navMesh = _localizer->getNavMesh();
		const NavMeshNode & node = navMesh->GetNode((unsigned int)currNode);
		const size_t OBST_COUNT = node.getObstacleCount();
		for (size_t o = 0; o < OBST_COUNT; ++o) {
			const NavMeshObstacle * obst = node.getObstacle(o);
			if (obst->pointOutside(pt)) {
				float distance = distSqPointLineSegment(obst->getP0(), obst->getP1(), pt);
				filter->FilterObstacle(obst, distance);
			}
		}

	}
}
