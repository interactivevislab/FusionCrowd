#pragma once

namespace TestFusionCrowd
{
	template<typename RunParams>
	class ITestCase
	{
	public:
		virtual void Pre() = 0;
		virtual void Run(const RunParams & params) = 0;
		virtual void Post() = 0;

		virtual ~ITestCase() { };
	};
}
