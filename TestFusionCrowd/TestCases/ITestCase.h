#pragma once

namespace TestFusionCrowd
{
	template<typename RunArgs>
	class ITestCase
	{
	public:
		virtual void Pre() = 0;

		virtual void Run(const RunArgs & args) = 0;

		virtual void Post() = 0;

		virtual ~ITestCase() { };
	};
}
