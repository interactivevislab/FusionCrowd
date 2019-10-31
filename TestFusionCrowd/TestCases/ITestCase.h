#pragma once

#include <string>

namespace TestFusionCrowd
{
	class ITestCase
	{
	public:
		virtual void Pre() = 0;
		virtual void Run() = 0;
		virtual void Post() = 0;

		virtual std::string GetName() const = 0;

		virtual ~ITestCase() { };
	};
}
