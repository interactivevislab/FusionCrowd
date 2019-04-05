#define LINKFUSIONCROWD_API __declspec(dllexport)

// Forward declarations
class Simulator;

struct agentInfo
{
	float* pos;
};

class FusionCrowdLinkUE4
{
#pragma warning(disable : 4996)
public:
	LINKFUSIONCROWD_API FusionCrowdLinkUE4();
	LINKFUSIONCROWD_API ~FusionCrowdLinkUE4();

	LINKFUSIONCROWD_API void StartFusionCrowd(char* naVMeshDir);
	LINKFUSIONCROWD_API int GetAgentCount();
	LINKFUSIONCROWD_API void AddAgent(int agentsCount);
	LINKFUSIONCROWD_API void GetPositionAgents(agentInfo* agentsPos);

private:
	Simulator* sim;
	int agentsCount;
	char* navMeshPath;
};