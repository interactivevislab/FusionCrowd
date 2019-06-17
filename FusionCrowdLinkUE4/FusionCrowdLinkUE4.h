#define LINKFUSIONCROWD_API __declspec(dllexport)

#include <memory>

#include "UE4StrategyProxy.h"

namespace FusionCrowd
{
	class Simulator;
	class NavMeshComponent;
	namespace Karamouzas
	{
		class KaramouzasComponent;
	}
	namespace ORCA
	{
		class ORCAComponent;
	}
	namespace PedVO
	{
		class PedVOComponent;
	}
}

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
	LINKFUSIONCROWD_API void UpdateNav(float x, float y);

private:
	FusionCrowd::Simulator* sim;
	std::shared_ptr<UE4StrategyProxy> _strategy;
	std::shared_ptr<FusionCrowd::Karamouzas::KaramouzasComponent> kComponent;
	std::shared_ptr<FusionCrowd::ORCA::ORCAComponent> orcaComponent;
	std::shared_ptr<FusionCrowd::PedVO::PedVOComponent> pedvoComponent;
	std::shared_ptr<FusionCrowd::NavMeshComponent> navMeshTactic;
	int agentsCount;
	char* navMeshPath;
};