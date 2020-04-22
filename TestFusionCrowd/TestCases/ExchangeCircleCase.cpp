#include "pch.h"

#include "ExchangeCircleCase.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	ExchangeCircleCase::ExchangeCircleCase(size_t agentsNum, size_t steps, ComponentId op, bool writeTraj)
		: ITestCase(agentsNum, steps, writeTraj), _op(op)
	{ }

	void ExchangeCircleCase::Pre()
	{
		std::shared_ptr<ISimulatorBuilder> builder(BuildSimulator(), BuilderDeleter);
		builder
			->WithNavMesh("Resources/square.nav")
			->WithOp(_op);

		_sim = std::shared_ptr<FusionCrowd::ISimulatorFacade>(builder->Build(), SimulatorFacadeDeleter);


		const float gap = 0.1f;
		const float bigR = _agentsNum * (2 * 0.19f + gap) / 6.28f;

		const float dAlpha = 6.28 / _agentsNum;

		for (size_t i = 0; i < _agentsNum; i++)
		{
			const float alpha = dAlpha * i;
			const float oppositeAlpha = alpha + 3.1415f;

			size_t id = _sim->AddAgent(bigR * cos(alpha), bigR * sin(alpha), _op, ComponentIds::NAVMESH_ID, ComponentIds::NO_COMPONENT);
			_sim->SetAgentGoal(id, Point { bigR * cos(oppositeAlpha), bigR * sin(oppositeAlpha) } );
		}
	}
}
