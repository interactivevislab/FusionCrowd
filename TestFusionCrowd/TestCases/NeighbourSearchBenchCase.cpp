#include "pch.h"
#include "NeighbourSearchBenchCase.h"

#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <time.h>
#include <ctime>
#include <chrono>

#include "Math/consts.h"
#include "Benchmark/MicroscopicMetrics.h"
#include "Export/Export.h"

#include "TestCases/Utils.h"

namespace TestFusionCrowd
{
	using namespace DirectX::SimpleMath;
	using namespace std::chrono;
	using namespace FusionCrowd;

	NeighbourSearchBenchCase::NeighbourSearchBenchCase(float coef) : ITestCase(5 * worldSide * worldSide, 100), _coef(coef)
	{
	}

	void NeighbourSearchBenchCase::Pre()
	{
		std::string navPath = "Resources/square.nav";

		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder->WithNavMesh("Resources/square.nav")
			->WithOp(FusionCrowd::ComponentIds::KARAMOUZAS_ID)
			->WithOp(FusionCrowd::ComponentIds::ORCA_ID);

		_sim = std::shared_ptr<ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);

		for (int i = 0; i < _agentsNum; i++)
		{
			auto id = _sim->AddAgent(RandFloat(0, worldSide), RandFloat(0, worldSide), FusionCrowd::ComponentIds::KARAMOUZAS_ID, ComponentIds::NAVMESH_ID, -1);
			_sim->SetAgentGoal(id, Point { RandFloat(0, worldSide), RandFloat(0, worldSide)});
		}
	}
}
