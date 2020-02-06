#pragma once

#include "StrategyComponent/Goal/Goal.h"
#include "GroupShape.h"

#include <vector>
#include <memory>
#include <unordered_set>

namespace FusionCrowd
{
	class Group
	{
	public:
		static const size_t NO_GROUP = 0;

	public:
		Group();

		Group(size_t id, size_t dummyId, std::unique_ptr<IGroupShape> shape);

		const size_t id;
		const size_t dummyAgentId;

		IGroupShape* GetShape() const;

		size_t GetSize() const;

		bool Contains(size_t agentId);

		void AddAgent(size_t agentId);
		void RemoveAgent(size_t agentId);

		std::vector<size_t> GetAgents();

	private:
		std::unique_ptr<IGroupShape> _shape;

		std::unordered_set<size_t> _agents;
	};
}
