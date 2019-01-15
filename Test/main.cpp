#include "Agents/SimulatorInterface.h"
#include "Math/RandGenerator.h"
#include "PluginEngine/CorePluginEngine.h"
#include "ProjectSpec.h"
#include "Runtime/Logger.h"
#include "Runtime/os.h"
#include "Runtime/SimulatorDB.h"
#include "Agents/BaseAgent.h"

#include "Agents/AgentInitializer.h"
#include "Agents/SpatialQueries/SpatialQuery.h"
#include "Agents/SpatialQueries/SpatialQueryFactory.h"
#include "Agents/SpatialQueries/SpatialQueryKDTree.h"
#include "Agents/ObstacleSets/ObstacleSetDatabase.h"
#include "Runtime/SimulatorDBEntry.h"
#include "Orca/ORCADBEntry.h"
#include "Agents/ProfileSelectors/ProfileSelector.h"
#include "Agents/AgentGenerators/RectGridGenerator.h"
#include "Agents/StateSelectors/StateSelector.h"
#include "Agents/AgentGenerators/AgentGenerator.h"
#include "Agents/ProfileSelectors/ConstProfileSelector.h"
#include "Agents/StateSelectors/ConstStateSelector.h"
#include "Agents/XMLSimulatorBase.h"
#include "Agents/SimulatorState.h"

#include "Core.h"
#include "Agents/BaseAgent.h"
#include "Agents/SimulatorState.h"
#include "Agents/XMLSimulatorBase.h"
#include "Agents/AgentGenerators/AgentGeneratorDatabase.h"
#include "Agents/Elevations/ElevationDatabase.h"
#include "Agents/ObstacleSets/ObstacleSetDatabase.h"
#include "Agents/ProfileSelectors/ProfileSelectorDatabase.h"
#include "Agents/SpatialQueries/SpatialQueryDatabase.h"
#include "Agents/StateSelectors/StateSelectorDatabase.h"
#include "Runtime/os.h"
#include "Agents/ObstacleSets/ExplicitObstacleSet.h"
#include "BFSM/FSM.h"

#include "BFSM/FSMDescrip.h"
#include "BFSM/StateDescrip.h"
#include "BFSM/GoalSet.h"
#include "BFSM/Actions/Action.h"
#include "BFSM//GoalSelectors/GoalSelectorMirror.h"
#include "BFSM/GoalSelectors/GoalSelectorIdentity.h"
#include "BFSM/VelocityComponents/VelCompGoal.h"
#include "BFSM/Transitions/Condition.h"
#include "BFSM/Transitions/Target.h"
#include "BFSM/Transitions/CondGoal.h"
#include "BFSM/Transitions/Transition.h"
#include "BFSM/FSM.h"

#include "Runtime/SimulatorDBEntry.h"

#include "MengeException.h"
#include "Agents/SimulatorInterface.h"
#include "BFSM/FSM.h"
#include "BFSM/FSMDescrip.h"
#include "Runtime/Logger.h"

#include "Orca/ORCAInitializer.h"

#include "Math/RandGenerator.h"
#include "Orca/ORCAAgent.h"

#include "Agents/AgentInitializer.h"

#include "Agents/BaseAgent.h"
#include "Math/RandGenerator.h"
#include "Runtime/Utils.h"


#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace Menge;

// Time step (in seconds)
float TIME_STEP = 0.2f;
// The number of uniform simulation steps to take between logical time steps
size_t SUB_STEPS = 0;
// Maximum duration of simulation (in seconds)
float SIM_DURATION = 800.f;
// Controls whether the simulation is verbose or not
bool VERBOSE = false;
// The location of the executable - for basic executable resources
std::string ROOT;
SimulatorDB simDB;

std::string getPluginPath()
{
#ifdef _WIN32
#ifdef NDEBUG
	return os::path::join(2, ROOT.c_str(), "plugins");
#else	// NDEBUG
	return os::path::join(3, ROOT.c_str(), "plugins", "debug");
#endif	// NDEBUG
#else	// _WIN32
	return os::path::join(2, ROOT.c_str(), "plugins");
#endif	// _WIN32
}

BFSM::FSM * BuildFSM(BFSM::FSMDescrip & fsmDescrip, Agents::SimulatorInterface * sim, bool VERBOSE)
{
	using namespace BFSM;
	using namespace Agents;
	SPATIAL_QUERY = sim->getSpatialQuery();

	//Global Simulator interface
	SIMULATOR = sim;

	bool valid = true;
	const size_t AGT_COUNT = sim->getNumAgents();
	FSM * fsm = new FSM(sim);

	// Build the fsm

	//we'll need this iterator later
	std::vector< VelModifier * >::iterator vItr;


	// Map of state names to state IDs.
	std::map< std::string, size_t > stateNameMap;

	// Copy the GoalSets
	fsm->_goalSets.clear();
	fsm->_goalSets.insert(fsmDescrip._goalSets.begin(), fsmDescrip._goalSets.end());
	fsmDescrip._goalSets.clear();

	//	1. Create states
	//		a. Add velocity components and actions
	//		b. add to fsm

	std::list< StateDescrip * >::const_iterator sItr = fsmDescrip._states.begin();
	for (; sItr != fsmDescrip._states.end(); ++sItr) {
		StateDescrip * sData = *sItr;
		State * s = fsmDescrip.addState(sData);

		if (s == 0x0) {
			logger << Logger::ERR_MSG << "Error creating state!";
			delete fsm;
			return 0x0;
		}
		if (VERBOSE) {
			logger << Logger::INFO_MSG << "\tAdding state: " << s->getName();
			logger << "(" << s->getID() << ")\n";
		}


		// State's goal selector
		GoalSelector * gs = sData->_goalSelector;
		if (gs == 0x0) {
			logger << Logger::WARN_MSG << "The state " << sData->_name;
			logger << " doesn't specify a goal selector.  "
				"The identity goal selector will be used.";
			gs = new IdentityGoalSelector();
		}

		try {
			gs->setGoalSet(fsm->getGoalSets());	// possibly throws GoalSelectorException
			s->setGoalSelector(gs);				// possibly throws GoalSelectorException
		}
		catch (GoalSelectorException) {
			logger << Logger::ERR_MSG << "Problem initializing the goal selector for "
				"the state " << s->getName() << ".";
			delete fsm;
			return 0x0;
		}
		sData->_goalSelector = 0x0;

		// construct each velocity component
		if (sData->_velComponent == 0x0) {
			logger << Logger::WARN_MSG << "The state " << sData->_name << " doesn't "
				"specify a velocity component.  The zero velocity component will be used.";
			s->setVelComponent(new ZeroVelComponent());
		}
		else {
			s->setVelComponent(sData->_velComponent);
			sData->_velComponent = 0x0;
		}

		// transfer each action
		std::list< Action * >::iterator aItr = sData->_actions.begin();
		for (; aItr != sData->_actions.end(); ++aItr) {
			s->addAction(*aItr);
		}
		sData->_actions.clear();

		//transfer velocity modifiers from the state description
		vItr = sData->_velModifiers.begin();
		for (; vItr != sData->_velModifiers.end(); ++vItr) {
			s->addVelModifier(*vItr);
		}
		sData->_velModifiers.clear();


		// Set start node
		size_t stateID = fsm->addNode(s);
		stateNameMap[sData->_name] = stateID;
	}

	// Connect all shared goal selectors
	std::map< std::string, size_t >::iterator stateItr = stateNameMap.begin();
	for (; stateItr != stateNameMap.end(); ++stateItr) {
		std::string stateName = stateItr->first;
		size_t stateID = stateItr->second;
		State * state = fsm->getNode(stateID);
		SharedGoalSelector * gs =
			dynamic_cast<SharedGoalSelector *>(state->getGoalSelector());
		if (gs != 0x0) {
			if (stateNameMap.count(gs->_stateName) == 0) {
				logger << Logger::ERR_MSG << "Found shared goal selector defined on line ";
				logger << gs->_lineNo << ", but unable to locate state with the provided "
					"name: \"" << gs->_stateName << "\".";
				delete fsm;
				return 0x0;
			}
			State * src = fsm->getNode(stateNameMap[gs->_stateName]);
			if (dynamic_cast<SharedGoalSelector *>(src->getGoalSelector())) {
				logger << Logger::ERR_MSG << "Shared goal selector defined on line ";
				logger << gs->_lineNo << " references a state with a shared goal.  The "
					"source state must have a full goal selector definition.";
				delete fsm;
				return 0x0;
			}
			state->clearGoalSelector();
			GoalSelector * srcGS = src->getGoalSelector();
			srcGS->setPersistence(true);
			state->setGoalSelector(srcGS);
		}
	}

	if (VERBOSE) {
		logger << Logger::INFO_MSG << "There are " << fsmDescrip._transitions.size();
		logger << " transitions\n";
	}

	//	2. Create transitions
	std::map< std::string, std::list< Transition * > >::iterator stItr =
		fsmDescrip._transitions.begin();
	for (; stItr != fsmDescrip._transitions.end(); ++stItr) {
		const std::string fromName = stItr->first;
		std::list< Transition * > & tList = stItr->second;

		// Determine if the origin state is valid
		if (fsmDescrip._stateNameMap.find(fromName) ==
			fsmDescrip._stateNameMap.end()) {
			logger << Logger::ERR_MSG << "Transition with invalid from node name: ";
			logger << fromName << ".";
			delete fsm;
			return 0x0;
		}

		// Try to connect the transitions to the destination(s)
		std::list< Transition * >::iterator tItr = tList.begin();
		for (; tItr != tList.end(); ++tItr) {
			Transition * t = *tItr;
			if (!t->connectStates(fsmDescrip._stateNameMap)) {
				delete fsm;
				return 0x0;
			}
			fsm->addTransition(stateNameMap[fromName], t);
		}
		tList.clear();
	}
	fsmDescrip._transitions.clear();

	//////////////

	// copy over the velocity modifiers
	vItr = fsmDescrip._velModifiers.begin();
	//TODO: replace global vel mod initalizer
	for (; vItr != fsmDescrip._velModifiers.end(); ++vItr) {
		fsm->addVelModifier(*vItr);
	}
	fsmDescrip._velModifiers.clear();

	// 3. Query simulator and fsm for possible reasons to have a task
	fsm->collectTasks();
	for (std::list< Task * >::iterator itr = fsmDescrip._tasks.begin();
		itr != fsmDescrip._tasks.end();
		++itr) {
		fsm->addTask((*itr));
	}
	fsmDescrip._tasks.clear();

	// spatial query and elevation tasks
	fsm->addTask(SPATIAL_QUERY->getTask());
	if (sim->getElevationInstance()) {
		// NOTE: The elevation instance is null because none were specified.
		//	Eventually, the default elevation will be set.
		// HOWEVER, if the default ever changes such that it requires a task,
		//	this won't catch it!!!  So, make sure the default never requires
		//	a task.
		fsm->addTask(sim->getElevationInstance()->getTask());
	}

	logger << Logger::INFO_MSG << "There are " << fsm->getTaskCount();
	logger << " registered tasks.\n";
	fsm->doTasks();



	//	5. Initialize all agents
	if (VERBOSE) logger << Logger::INFO_MSG << "Initializing agents:\n";
	Agents::SimulatorState * initState = sim->getInitialState();

	for (size_t a = 0; a < AGT_COUNT; ++a) {
		Agents::BaseAgent * agt = sim->getAgent(a);
		// update current state to class-appropriate value
		const std::string stateName = initState->getAgentState(agt->_id);

		std::map< std::string, size_t >::iterator stateIDItr =
			stateNameMap.find(stateName);
		if (stateIDItr == stateNameMap.end()) {
			logger << Logger::ERR_MSG << "Agent " << agt->_id;
			logger << " requested to start in an unknown state: " << stateName << ".";
			delete fsm;
			return 0x0;
		}
		size_t stateID = stateIDItr->second;

		// initialize velocity to preferred velocity
		State * cState = fsm->getNode(stateID);
		if (VERBOSE) {
			logger << Logger::INFO_MSG << "Agent " << agt->_id << " starts in ";
			logger << cState->getName() << ".";
		}
		fsm->setCurrentState(agt, stateID);
		cState->enter(agt);
		// TODO: Restore support for defining inital velocity state: zero or preferred
		agt->_vel.set(Vector2(0.f, 0.f));

		//register the agent for all vel modifiers
		vItr = fsm->_velModifiers.begin();
		//TODO: replace global vel mod initalizer
		for (; vItr != fsm->_velModifiers.end(); ++vItr) {
			(*vItr)->registerAgent(agt);
		}
	}

	ACTIVE_FSM = fsm;
	return fsm;
}

bool finalize(Agents::SimulatorInterface * sim, BFSM::FSM * fsm)
{
	using namespace BFSM;
	using namespace Agents;
	sim->setBFSM(fsm);
	// older versions of OpenMP require signed for loop counters
	int agtCount = (int)sim->getNumAgents();
#pragma omp parallel for
	for (int a = 0; a < agtCount; ++a)
	{
		Agents::BaseAgent * agt = sim->getAgent(a);
		fsm->computePrefVelocity(agt);
	}
	try {
		sim->finalize();
	}
	catch (Menge::MengeException & e)
	{
		logger << Logger::ERR_MSG << "Problem in finalizing the simulator.\n";
		logger << "\t" << e.what();
		delete fsm;
		return false;
	}
	try
	{
		fsm->finalize();
	}
	catch (Menge::MengeFatalException & e)
	{
		logger << Logger::ERR_MSG << "Fatal error finalizing the finite state machine!\n";
		logger << "\t" << e.what();
		delete fsm;
		return false;
	}
	catch (Menge::MengeException & e)
	{
		logger << Logger::WARN_MSG << "There were non-fatal errors in finalizing the finite "
			"state machine!\n";
		logger << "\t" << e.what();
	}
	return true;
}

int simMain(SimulatorDBEntry * dbEntry, const std::string & outFile) {
	size_t agentCount;
	if (outFile != "") {
		logger << Logger::INFO_MSG << "Attempting to write scb file: " << outFile << "\n";
	}

	using Menge::Agents::SimulatorInterface;

	//SimulatorInterface * sim = dbEntry->getSimulator(agentCount, TIME_STEP, SUB_STEPS,
	//	SIM_DURATION, behaveFile, sceneFile, outFile,
	//	scbVersion, VERBOSE);

	HASH_MAP< std::string, Agents::AgentInitializer * >	_profiles;

	SimulatorInterface* simTest = dbEntry->getNewSimulator();

	Agents::AgentInitializer * agentInit = dbEntry->getAgentInitalizer();
	Agents::XMLSimulatorBase* simXMLTest(simTest);


	simXMLTest->setExpParam("time_step", "0.1");

	// для GCF
	//simXMLTest->setExpParam("reaction_time", "0.5");
	//simXMLTest->setExpParam("max_agent_dist", "2");
	//simXMLTest->setExpParam("max_agent_force", "3");
	//simXMLTest->setExpParam("agent_interp_width", "0.1");
	//simXMLTest->setExpParam("agent_force_strength", "0.35");

	Agents::SpatialQuery * spQuery = new Agents::BergKDTree();
	simXMLTest->setSpatialQuery(spQuery);


	Agents::AgentInitializer * init;
	init = agentInit->copy();
	init->setDefaults();
	_profiles["group1"] = init;
	init->constFloatGenerator(init->_neighborDist, "5");
	init->constFloatGenerator(init->_prefSpeed, "1.34");
	init->constFloatGenerator(init->_maxSpeed, "10");
	init->constFloatGenerator(init->_maxAccel, "5");
	init->constIntGenerator(init->_maxNeighbors, "10");
	init->constFloatGenerator(init->_radius, "0.2");
	init->constFloatGenerator(init->_maxAngVel, "360", DEG_TO_RAD);
	init->constSizet(init->_obstacleSet, "1");
	//init->constSizet(init->_class, "");
	//init->constFloat(init->_priority, "");


	//ORCA::AgentInitializer * initORCA = new ORCA::AgentInitializer();
	//initORCA->setDefaults();
	//initORCA->getFloatGenerator(initORCA->_timeHorizon, "3.0");
	//initORCA->getFloatGenerator(initORCA->_timeHorizonObst, "0.15");

	Agents::ConstProfileSelector * profileSel = new Agents::ConstProfileSelector();
	profileSel->setName("group1");
	if (!profileSel->cacheProfiles(_profiles)) {
		logger << Logger::ERR_MSG << "ProfileSelector on line ";
		logger << " was unable to find a named profile.";
		return false;
	}

	Agents::ConstStateSelector * stateSel = new Agents::ConstStateSelector();
	stateSel->setStateName("Walk");
	Agents::RectGridGenerator * generator = new Agents::RectGridGenerator();
	FloatGenerator * gen = new UniformFloatGenerator(0, 0.25, -3);
	generator->setNoiseGenerator(gen);
	generator->setAnchor(Menge::Vector2(-7.3, -7.3));
	generator->setOffset(Menge::Vector2(-1.333, -1.333));
	generator->setXCount(5);
	generator->setYCount(5);
	generator->setRotationDeg(6);

	const size_t AGT_COUNT = generator->agentCount();
	Vector2 zero;

	for (size_t i = 0; i < AGT_COUNT; ++i)
	{
		Agents::BaseAgent * agent = simTest->addAgent(zero, profileSel->getProfile());
		generator->setAgentPosition(i, agent);
		simXMLTest->getInitialState()->setAgentState(agent->_id, stateSel->getState());
	}

	generator->destroy();

	Agents::ExplicitObstacleSet* obTest = new Agents::ExplicitObstacleSet();
	obTest->setClass(1);
	Agents::ObstacleVertexList obs;
	obs.closed = true;
	obs.vertices.push_back(Vector2(1.333, 1.333));
	obs.vertices.push_back(Vector2(5.333, 1.333));
	obs.vertices.push_back(Vector2(5.333, 5.333));
	obs.vertices.push_back(Vector2(1.333, 5.333));
	obTest->addObstacle(obs);

	obs.vertices.clear();
	obs.vertices.push_back(Vector2(-5.333, 1.333));
	obs.vertices.push_back(Vector2(-1.333, 1.333));
	obs.vertices.push_back(Vector2(-1.333, 5.333));
	obs.vertices.push_back(Vector2(-5.333, 5.333));
	obTest->addObstacle(obs);

	obs.vertices.clear();
	obs.vertices.push_back(Vector2(-5.333, -5.333));
	obs.vertices.push_back(Vector2(-1.333, -5.333));
	obs.vertices.push_back(Vector2(-1.333, -1.333));
	obs.vertices.push_back(Vector2(-5.333, -1.333));
	obTest->addObstacle(obs);

	obs.vertices.clear();
	obs.vertices.push_back(Vector2(1.333, -5.333));
	obs.vertices.push_back(Vector2(5.333, -5.333));
	obs.vertices.push_back(Vector2(5.333, -1.333));
	obs.vertices.push_back(Vector2(1.333, -1.333));
	obTest->addObstacle(obs);

	for (int index = 0; index < obTest->obstacleCount(); ++index) {
		simXMLTest->getSpatialQuery()->addObstacle(obTest->getObstacle(index));
	}
	if (!simXMLTest->initSpatialQuery())
	{
		return 1;
	}

	BFSM::FSMDescrip fsmDescrip;

#pragma region FSM State
	BFSM::StateDescrip * s = new BFSM::StateDescrip("Walk", false);
	BFSM::MirrorGoalSelector* mgl = new BFSM::MirrorGoalSelector();
	mgl->setMirror(1, 1);
	s->_goalSelector = mgl;
	BFSM::GoalVelComponent* gvl = new BFSM::GoalVelComponent();
	s->_velComponent = gvl;

	BFSM::StateDescrip * s1 = new BFSM::StateDescrip("Stop", true);
	BFSM::IdentityGoalSelector* mgl1 = new BFSM::IdentityGoalSelector();
	s1->_goalSelector = mgl1;
	BFSM::GoalVelComponent* gvl1 = new BFSM::GoalVelComponent();
	s1->_velComponent = gvl1;

	fsmDescrip._states.push_back(s);
	fsmDescrip._states.push_back(s1);
#pragma endregion

#pragma region FSM Transition
	BFSM::GoalCondition * condition = new BFSM::GoalCondition();
	condition->setMinDistance(0.05);
	BFSM::TransitionTarget * target = new BFSM::SingleTarget("Stop");
	BFSM::Transition* transition = new BFSM::Transition(condition, target);
	fsmDescrip.addTransition("Walk", transition);
#pragma endregion
	BFSM::FSM * fsm = BuildFSM(fsmDescrip, simTest, false);

	if (!finalize(simTest, fsm)) {
		delete simTest;
		delete fsm;
		return 1;
	}
	simTest->setSubSteps(0);
	simTest->setMaxDuration(400);
	agentCount = simTest->getNumAgents();

	bool running = true;

	Menge::Agents::BaseAgent * a;
	while (running)
	{
		running = simTest->step();
		for (int i = 0; i < agentCount; i++)
		{
			a = simTest->getAgent(i);
			Menge::Math::Vector2 ac = a->_pos;
			std::cout << i << " " << ac._x << " " << ac._y << "\n";
		}
	}


	//if (sim == 0x0)
	//{
	//	return 1;
	//}

	//bool running = true;

	//Menge::Agents::BaseAgent * a;
	//while (running)
	//{
	//	running = sim->step();
	//	for (int i = 0; i < agentCount; i++)
	//	{
	//		a = sim->getAgent(i);
	//		Menge::Math::Vector2 ac = a->_pos;
	//		std::cout << i << " " << ac << "\n";
	//	}
	//}

	return 0;
}

int main()
{
	logger.setFile("log.html");
	logger << Logger::INFO_MSG << "initialized logger";

	Menge::PluginEngine::CorePluginEngine plugins(&simDB);

	std::string pluginPath = "D:/Lebin/Project/FusionCrowd/x64/Debug//Plugins";
	plugins.loadPlugins(pluginPath);
	int modelCount = simDB.modelCount();
	if (simDB.modelCount() == 0)
	{
		logger << Logger::INFO_MSG << "There were no pedestrian models in the plugins folder\n";
		return 1;
	}
	ProjectSpec projSpec;


	VERBOSE = projSpec.getVerbosity();
	TIME_STEP = projSpec.getTimeStep();
	SUB_STEPS = projSpec.getSubSteps();
	SIM_DURATION = projSpec.getDuration();
	std::string dumpPath = projSpec.getDumpPath();
	Menge::Math::setDefaultGeneratorSeed(projSpec.getRandomSeed());
	std::string outFile = projSpec.getOutputName();

	std::string viewCfgFile = projSpec.getView();
	bool useVis = viewCfgFile != "";
	std::string model = "gcf";

	SimulatorDBEntry * simDBEntry = simDB.getDBEntry(model);
	if (simDBEntry == 0x0)
	{
		std::cerr << "!!!  The specified model is not recognized: " << model << "\n";
		logger.close();
		return 1;
	}

	int result = simMain(simDBEntry, projSpec.getOutputName());

	if (result) {
		std::cerr << "Simulation terminated through error.  See error log for details.\n";
	}
	logger.close();

	return 0;
}