#pragma once

#include <memory>
#include <string>

#include "Simulator.h"
#include "TestCases/ITestCase.h"
#include "TestCases/EIFPD/EIFPDDataset.h"

namespace TestFusionCrowd
{
	class EIFPDTestCase : public ITestCase<int>
	{
	public:
		EIFPDTestCase(std::string datasetPath);

		void Pre() override;

		void Run(const int& arg) override;

		void Post() override;

		~EIFPDTestCase();

	private:
		std::unique_ptr<EIFPDDataset> dataset;
		std::unique_ptr<FusionCrowd::Simulator> m_simulator;
	};
}
