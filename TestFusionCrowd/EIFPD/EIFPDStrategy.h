#pragma once

#include "Config.h"
#include "Simulator.h"
#include "StrategyComponent/IStrategyComponent.h"

namespace TestFusionCrowd
{
	class FUSION_CROWD_API EIFPDStrategy : public FusionCrowd::IStrategyComponent
	{
	public:
		EIFPDStrategy(FusionCrowd::Simulator & sim);

		void AddAgent(size_t id) override;
		bool RemoveAgent(size_t id) override;
		void Update(float timeStep) override;
		std::string GetName() override;

		~EIFPDStrategy();
	};
}
