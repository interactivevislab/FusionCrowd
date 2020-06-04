#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"
#include "Math/Util.h"

#include <map>

namespace FusionCrowd
{
	namespace Bicycle
	{
		struct AgentParamentrs
		{
			float _theta;
			float _delta;
			float _length;
			float _orintX;
			float _orintY;
			float _acceleration;
			float _negativeAcceleration;

			AgentParamentrs() : _theta(0.0f), _delta(0.0f), _length(2.0f), _orintX(1.0f), _orintY(0.0f), _acceleration(0.5f), _negativeAcceleration(0.5f)
			{
			}
		};

		class BicycleComponent : public IOperationComponent
		{
		public:
			BicycleComponent(std::shared_ptr<NavSystem> navSystem);

			ComponentId GetId() override { return ComponentIds::BICYCLE; };

			void AddAgent(size_t id) override;
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			void ComputeNewVelocity(AgentSpatialInfo & spatialInfo, float timeStep);

			float CalcTargetSteeringRadius(const AgentParamentrs & agent, const AgentSpatialInfo & spatialInfo, DirectX::SimpleMath::Vector2 targetPoint);

			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;

			float _maxSteeringR = 1e7f;
		};
	}
}