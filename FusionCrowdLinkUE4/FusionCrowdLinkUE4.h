#define LINKFUSIONCROWD_API __declspec(dllexport)

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
};

class FusionCrowdLinkUE4
{
#pragma warning(disable : 4996)
public:
	LINKFUSIONCROWD_API FusionCrowdLinkUE4();
	LINKFUSIONCROWD_API ~FusionCrowdLinkUE4();

	LINKFUSIONCROWD_API void StartFusionCrowd(char* navMeshDir);
	LINKFUSIONCROWD_API int GetAgentCount();
	LINKFUSIONCROWD_API void AddAgents(int agentsCount);
	LINKFUSIONCROWD_API void GetPositionAgents(agentInfo* agentsPos);

private:
	FusionCrowd::Simulator* sim;
	FusionCrowd::Karamouzas::KaramouzasComponent* kComponent;
	FusionCrowd::ORCA::ORCAComponent* orcaComponent;
	FusionCrowd::NavMeshComponent* navMeshTactic;
	int agentsCount;
	char* navMeshPath;
};