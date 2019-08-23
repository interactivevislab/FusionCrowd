#include "pch.h"

#include "EIFPDTestCase.h"

using namespace FusionCrowd;

namespace TestFusionCrowd
{
	EIFPDTestCase::EIFPDTestCase(std::string datasetPath)
	{
		dataset = std::make_unique<EIFPDDataset>(datasetPath);
	}

	void EIFPDTestCase::Pre()
	{
		m_simulator = std::make_unique<Simulator>();

		m_simulator->SetNavSystem(dataset->GetNavSystem());
		m_simulator->AddStrategyComponent(std::make_shared<EIFPDStrategy>(dataset->GetStrategy()));

		//m_simulator->InitSimulator(dataset->GetConfig().NavMeshPath);
	}

	void EIFPDTestCase::Run(const int & arg)
	{
		float max_time = dataset->GetRecording().GetTimeSpan()[-1];

		while(m_simulator->GetElapsedTime() < max_time)
		{
			m_simulator->DoStep();
		}
	}

	void EIFPDTestCase::Post()
	{

	}

	EIFPDTestCase::~EIFPDTestCase()
	{

	}
}
