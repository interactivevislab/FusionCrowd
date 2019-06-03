#define LINKFUSIONCROWD_API __declspec(dllexport)

#include <memory>

// Forward declarations
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

	LINKFUSIONCROWD_API void SetOperationModel(size_t agentIdconst, const char * name);
	LINKFUSIONCROWD_API void StartFusionCrowd(char* navMeshDir);
	LINKFUSIONCROWD_API int GetAgentCount();
	LINKFUSIONCROWD_API void AddAgents(int agentsCount);
	LINKFUSIONCROWD_API void GetPositionAgents(agentInfo* agentsPos);

private:
	FusionCrowd::Simulator* sim;
	std::shared_ptr<FusionCrowd::Karamouzas::KaramouzasComponent> kComponent;
	std::shared_ptr<FusionCrowd::ORCA::ORCAComponent> orcaComponent;
	std::shared_ptr<FusionCrowd::NavMeshComponent> navMeshTactic;
	int agentsCount;
	char* navMeshPath;
};