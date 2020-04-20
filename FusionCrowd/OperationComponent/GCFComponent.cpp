#include "GCFComponent.h"

#include "Math/geomQuery.h"

#include <algorithm>

namespace FusionCrowd
{
	namespace GCF
	{
		using namespace DirectX::SimpleMath;

		float hermite_interp(float t, float x1, float x2, float y1, float y2, float dy1, float dy2) {
			assert(t >= x1 && t <= x2 && "Can only interpolate values inside the range");
			assert(x2 > x1 && "Intervals must be defined as x1 < x2");

			float scale = x2 - x1;
			t = (t - x1) / scale;
			float t2 = t * t;
			float t3 = t2 * t;
			float h1 = 2 * t3 - 3 * t2 + 1;
			float h2 = -2 * t3 + 3 * t2;
			float h3 = t3 - 2 * t2 + t;
			float h4 = t3 - t2;
			float left = y1 * h1 + dy1 * h3 * scale;
			float right = y2 * h2 + dy2 * h4 * scale;
			return left + right;
		}

		GCFComponent::GCFComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem)
		{
			_reactionTime = 0.5f;
			_nuAgent = 0.3f;
			_maxAgentDist = 2.0f;
			_maxAgentForse = 3.0f;
			_agentInterpWidth = 0.1f;
			_speedColor = false;
		}

		GCFComponent::GCFComponent(std::shared_ptr<NavSystem> navSystem, float reactionTime, float nuAgent, float maxAgentDist, float maxAgentForse, float agentInterpWidth, bool speedColor)
			: _navSystem(navSystem)
		{
			_reactionTime = reactionTime;
			_nuAgent = nuAgent;
			_maxAgentDist = maxAgentDist;
			_maxAgentForse = maxAgentForse;
			_agentInterpWidth = agentInterpWidth;
			_speedColor = speedColor;
		}


		GCFComponent::~GCFComponent()
		{
		}

		bool GCFComponent::DeleteAgent(size_t id)
		{
			if(_agents.find(id) == _agents.end())
				return false;

			_agents.erase(id);

			return true;
		}

		void GCFComponent::Update(float timeStep)
		{
			_timeStep = timeStep;
			for(auto & pair : _agents)
			{
				auto & info = _navSystem->GetSpatialInfo(pair.first);
				UpdateEllipse(info);
			}

			for(auto & pair : _agents)
			{
				auto & info = _navSystem->GetSpatialInfo(pair.first);

				ComputeNewVelocity(info);
			}
		}

		void GCFComponent::AddAgent(size_t agentId)
		{
			_agents[agentId] = AgentParamentrs();
			_navSystem->GetSpatialInfo(agentId).inertiaEnabled = true;
			UpdateEllipse(_navSystem->GetSpatialInfo(agentId));
		}

		void GCFComponent::ComputeNewVelocity(AgentSpatialInfo & agentInfo)
		{
			Vector2 force(DriveForce(agentInfo));  // driving force

			for (const auto & other : _navSystem->GetNeighbours(agentInfo.id))
			{
				float effDist, K_ij, response, velScale, magnitude;
				Vector2 forceDir;
				if (GetRepulsionParameters(agentInfo, other, effDist, forceDir, K_ij, response, velScale, magnitude) == 0)
				{
					force += forceDir * magnitude;
				}
			}

			// Obstacles
			const float SPEED = agentInfo.GetVel().Length();
			if (SPEED > 0.0001f) {
				// No obstacle force if basically stationary
				for (Obstacle & obst : _navSystem->GetClosestObstacles(agentInfo.id))
				{
					force += ObstacleForce(agentInfo, obst);
				}
			}

			// We're assuming unit mass
			agentInfo.velNew = agentInfo.GetVel() + force * _timeStep;
		}

		void GCFComponent::UpdateEllipse(const AgentSpatialInfo & agentInfo)
		{
			float speed = agentInfo.GetVel().Length();

			AgentParamentrs & agentParams = _agents[agentInfo.id];
			// update ellipse
			agentParams._ellipse.SetCenter(agentInfo.GetPos());
			agentParams._ellipse.SetOrientation(agentInfo.GetOrient());
			// compute major and minor axis values based on speed
			float major = agentParams._aMin + agentParams._aRate * speed;
			float minor = agentParams._bMax - agentParams._bGrowth * speed / 1.3f;
			agentParams._ellipse.SetAxes(major, minor);
		}

		Vector2 GCFComponent::DriveForce(const AgentSpatialInfo & agentInfo) const
		{
			return (agentInfo.prefVelocity.getPreferredVel() - agentInfo.GetVel()) / _reactionTime;
		}

		int GCFComponent::GetRepulsionParameters(
			const AgentSpatialInfo & agent, const NeighborInfo & other,
			float& effDist, DirectX::SimpleMath::Vector2& forceDir,
			float& K_ij, float& response, float& velScale, float& magnitude) const
		{
			const float PREF_SPEED = agent.prefVelocity.getPreferredVel().Length();

			const AgentParamentrs & agentParams = _agents.at(agent.id);
			const AgentParamentrs & otherParams = _agents.at(other.id);

			forceDir = agentParams._ellipse.ellipseCenterDisplace(otherParams._ellipse);
			float centerDist = forceDir.Length();
			float dca = agentParams._ellipse.DistanceOfClosestApproach(otherParams._ellipse);
			effDist = centerDist - dca;

			float dist = forceDir.Length();
			assert(dist > 0.0001f && "Agents are on top of each other");
			forceDir /= dist;

			if (effDist >= _maxAgentDist) {
				// Agent too far away to give force.
				return 1;
			}

			// field of view
			K_ij = agent.GetOrient().Dot(forceDir);

			// This represents 360 degree sensitivity, with the maximum sensitivity in the oriented
			//	direction fading to zero in the opposite direction
			//  remap [-1, 1] -> [-1, -0.1]
			K_ij = (K_ij * 0.45f) - 0.55f;

			// relative velocities
			Vector2 relVel = agent.GetVel() - other.vel;

			float velWeight = relVel.Dot(forceDir);
			velScale = _nuAgent * PREF_SPEED;
			if (velWeight <= 0.f) {
				// convergent velocity needs some extra pushing
				velScale -= velWeight / std::max(effDist, 0.01f);
			}

			// force response
			response = ComputeDistanceResponse(effDist);

			magnitude = (-K_ij * response * velScale * velScale);

			return 0;
		}

		Vector2 GCFComponent::ObstacleForce(const AgentSpatialInfo & agent, Obstacle & obst) const
		{
			Vector2 force(0.f, 0.f);

			if (obst.length() < 0.1f) {
				return force;  // ignore short obstacles
			}
			// force from three points: nearest point, and point along wall in front
			//	and point along wall behind.
			Vector2 nearPt;  // gets set by distanceSqToPoint
			float distSq;    // gets set by distanceSqToPoint
			if (obst.distanceSqToPoint(agent.GetPos(), nearPt, distSq) == Obstacle::LAST)
			{
				return force;
			}

			// No force if the agent is ON the point
			if (distSq < 0.0001f) return force;

			Vector2 disp = nearPt - agent.GetPos();
			float dist = sqrtf(distSq);
			Vector2 dir = disp / dist;

			// NOTE: An agent walking parallel with a wall does not *see* the wall and won't be pushed
			// away.  This makes *no* sense.  Even from a vision perspective, this doesn't make sense
			// if the wall extends out in *front* of the agent.
			// test visibility
			float cosTheta = agent.GetOrient().Dot(dir);
			// No force if the point is more than 90 degrees away from
			//	movement direction
			if (cosTheta < -0.5f) return force;

			// This is an APPROXIMATION of the actual distance to the wall
			float boundDist = _agents.at(agent.id)._ellipse.ApproximateMinimumDistance(nearPt);
			float Bij = 1.f - dist / boundDist;

			//// No force if the point lies inside the ellipse
			if (Bij > 0.f) return force;

			const float PREF_SPEED = agent.prefVelocity.getPreferredVel().Length();
			force = dir * Bij * PREF_SPEED;

			return force;
		}

		float GCFComponent::ComputeDistanceResponse(float effDist) const
		{
			//	Evaluates a piece-wise hermite curve based on the value of effDist
  //		The boundaries of the function are defined by user parameters.
  //
  //          0.0        interpWidth          maxDist-interpWidth   maxDist
  //	     ----|-------------|--------------------------|--------------|----
  //         5   |     4       |            3             |      2       | 1
			const float maxDist = _maxAgentDist;
			const float interpWidth = _agentInterpWidth;
			const float maxForce = _maxAgentForse;

			// Distance to great to make a difference - region 1
			if (effDist >= maxDist) {
				return 0.f;
			}

			// Distance so close that a constant force should be applied. - region 5
			if (effDist <= 0) {
				return 3.f * maxForce;
			}

			// Aproaching maximum distance with linearly decreasing force - region 2
			const float dist_intpol_right = maxDist - interpWidth;
			if (effDist > dist_intpol_right) {
				float f = 1.f / dist_intpol_right;
				float fDeriv = -f * f;
				return hermite_interp(effDist, dist_intpol_right, maxDist, f, 0.f, fDeriv, 0.f);
			}

			// middle domain - simply inverse force - region 3
			if (effDist > interpWidth) {
				return 1.f / effDist;
			}
			// closest domain, smoothing converge to constant - region 4
			float f = 1.f / interpWidth;
			float fDeriv = -f * f;
			return hermite_interp(effDist, 0, interpWidth, 3.f * maxForce, f, 0, fDeriv);
		}
	}
}
