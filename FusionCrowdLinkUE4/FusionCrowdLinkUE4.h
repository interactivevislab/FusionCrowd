#define LINKFUSIONCROWD_API __declspec(dllexport)

#include <memory>

#include "UE4StrategyProxy.h"
#include "Export.h"
#include "StrategyComponent/IStrategyComponent.h"

struct agentInfo
{
	size_t id;
	float* pos;
	float* orient;
	float* vel;
	float radius;
	char* opCompName;
};

class FusionCrowdLinkUE4
{
#pragma warning(disable : 4996)
public:
	LINKFUSIONCROWD_API FusionCrowdLinkUE4();
	LINKFUSIONCROWD_API ~FusionCrowdLinkUE4();

	LINKFUSIONCROWD_API void SetGoal(size_t agentId, const float * goalPos);
	LINKFUSIONCROWD_API void SetOperationModel(size_t agentId, const char * name);
	LINKFUSIONCROWD_API void StartFusionCrowd(char* navMeshDir);
	LINKFUSIONCROWD_API int GetAgentCount();
	LINKFUSIONCROWD_API size_t AddAgent(const float * agentPos, const float * goalPos, const char * opComponent);
	LINKFUSIONCROWD_API void AddAgents(int agentsCount);
	LINKFUSIONCROWD_API void GetPositionAgents(agentInfo* agentsPos);

private:
	FusionCrowd::IStrategyComponent* ProxyStrategyFactory(FusionCrowd::ISimulatorFacade* simFacade);

	FusionCrowd::ComponentId strategyId;
	std::shared_ptr<FusionCrowd::ISimulatorFacade> sim;
	UE4StrategyProxy * _strategy;

	int agentsCount;
	char* navMeshPath;
};