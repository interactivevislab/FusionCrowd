#pragma once

#include "Export/Export.h"

#include <memory>


namespace FusionCrowdWeb
{
	using ISimulator = FusionCrowd::ISimulatorFacade;

	namespace
	{
		template<typename T>
		T ReverseRead(const char*& source, T dest)
		{
			source -= sizeof(T);
			memcpy_s(&dest, sizeof(T), source, sizeof(T));
			return dest;
		}

		inline size_t SizeOf()
		{ 
			return 0;
		}

		template <typename Head, typename... Tail>
		inline size_t SizeOf(const Head & head, const Tail &... tail)
		{
			return sizeof(head) + SizeOf(tail...);
		}
	}

	class IRequestProcessor
	{
	public:
		virtual size_t GetInputSize() = 0;
		virtual size_t GetOutputSize() = 0;
		virtual void Process(std::shared_ptr<ISimulator> simulator, const char* args, char* outResult) = 0;
	};


	template<typename ReturnType, typename... ArgsTypes>
	class RequestProcessor : public IRequestProcessor
	{
	public:
		typedef ReturnType(ISimulator::* SimulatorMethodSignature) (ArgsTypes...);

		RequestProcessor(SimulatorMethodSignature simulatorMethod) : _simulatorMethod(simulatorMethod) {};
		~RequestProcessor() {};

		size_t GetInputSize() override
		{
			return SizeOf(ArgsTypes()...);
		}

		size_t GetOutputSize() override
		{
			return sizeof(ReturnType);
		}

		void Process(std::shared_ptr<ISimulator> simulator, const char* args, char* outResult) override
		{
			args += SizeOf(ArgsTypes()...);
			ReturnType result = ((*simulator).*_simulatorMethod)((ReverseRead(args, ArgsTypes()))...);
			memcpy_s(outResult, sizeof(ReturnType), &result, sizeof(ReturnType));
		}

	private:
		SimulatorMethodSignature _simulatorMethod;
	};


	template<typename... ArgsTypes>
	class RequestProcessor<void, ArgsTypes...> : public IRequestProcessor
	{
	public:
		typedef void (ISimulator::* SimulatorMethodSignature) (ArgsTypes...);

		RequestProcessor(SimulatorMethodSignature simulatorMethod) : _simulatorMethod(simulatorMethod) {};
		~RequestProcessor() {};

		size_t GetInputSize() override
		{
			return SizeOf(ArgsTypes()...);
		}

		size_t GetOutputSize() override
		{
			return 0;
		}

		void Process(std::shared_ptr<ISimulator> simulator, const char* args, char* outResult) override
		{
			args += SizeOf(ArgsTypes()...);
			((*simulator).*_simulatorMethod)((ReverseRead(args, ArgsTypes()))...);
		}

	private:
		SimulatorMethodSignature _simulatorMethod;
	};
}