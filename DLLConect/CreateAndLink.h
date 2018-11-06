#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <stdio.h>

#ifdef LINKMENGECORE_EXPORTS
#define LINKMENGE_API __declspec(dllexport)
#else
#define LINKMENGE_API __declspec( dllimport )
#endif // MENGEEXPORT_API


#pragma warning(push)
#pragma warning(disable: 4251)

namespace  Menge
{
	namespace Agents
	{
		class SimulatorInterface;
	}
}
namespace Menge
{
	class SimulatorDB;
	class SimulatorDBEntry;
}
namespace Menge
{
	class ProjectSpec;
}
namespace Menge
{
	namespace BFSM
	{
		class  FSM;
		class  FSMDescrip;
	}
}


__declspec(dllexport) float TIME_STEP = 0.2f;
__declspec(dllexport) size_t SUB_STEPS = 0;
__declspec(dllexport) float SIM_DURATION = 800.f;
__declspec(dllexport) bool VERBOSE = false;
__declspec(dllexport) std::string ROOT;

__declspec(dllexport) std::string pathProject;

__declspec(dllexport) size_t agentCount;

__declspec(dllexport) bool isDisposed = false;

__declspec(dllexport) Menge::SimulatorDB* simDB;
__declspec(dllexport) Menge::ProjectSpec* projSpec;
__declspec(dllexport) Menge::Agents::SimulatorInterface* simTest;

__declspec(dllexport) class CreateAndLink
{
private:
	// Time step (in seconds)
	// The number of uniform simulation steps to take between logical time steps
	// Maximum duration of simulation (in seconds)
	// Controls whether the simulation is verbose or not
	// The location of the executable - for basic executable resources
	

	static Menge::BFSM::FSM * BuildFSM(Menge::BFSM::FSMDescrip & fsmDescrip, Menge::Agents::SimulatorInterface * sim, bool VERBOSE);
	static bool finalize(Menge::Agents::SimulatorInterface * sim, Menge::BFSM::FSM * fsm);
	static int simMain(Menge::SimulatorDBEntry * dbEntry, const std::string & outFile);

#pragma warning(pop)
public:
	__declspec(dllexport) CreateAndLink();
	__declspec(dllexport) ~CreateAndLink();

	__declspec(dllexport) static void Init();
	__declspec(dllexport) static int GetAgentCount();
	__declspec(dllexport) static float* Update();
};


extern "C" __declspec(dllexport) int* InitSim()
{
	CreateAndLink::Init();
	return 0;
}

extern "C" __declspec(dllexport) float* getPos()
{
	return CreateAndLink::Update();
}

extern "C" __declspec(dllexport) int GetAgentCount()
{
	return CreateAndLink::GetAgentCount();
}
