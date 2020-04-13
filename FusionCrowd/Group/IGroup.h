#pragma once

#include <vector>
#include <memory>
#include <unordered_set>

#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class IGroup
	{
	public:
		static const size_t NO_GROUP = 0;

	protected:
		IGroup(size_t id, size_t dummyId);

	public:
		virtual size_t GetSize() const = 0;
		virtual bool Contains(size_t agentId) const = 0;
		virtual void AddAgent(size_t agentId, AgentSpatialInfo& info) = 0;
		virtual void RemoveAgent(size_t agentId) = 0;
		virtual std::vector<size_t> GetAgents() const = 0;
		virtual DirectX::SimpleMath::Vector2 GetRelativePos(size_t agentId) const = 0;
		virtual float GetRadius() const = 0;

		void SetAgentPrefVelocity(const AgentSpatialInfo & dummyInfo, AgentSpatialInfo & agentInfo, float timeStep) const;

		size_t GetDummyId() const;
		size_t GetId() const;
	private:
		size_t _id;
		size_t _dummyAgentId;
	};
}
