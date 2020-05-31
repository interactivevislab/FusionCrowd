#pragma once

#include "FcMainServerApi.h"
#include "WebNode.h"
#include "Export/Export.h"


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_MAIN_SERVER_API FcMainServer
	{
	public:
		void StartOn(const char* inIpAdress, short inPort);
		void Shutdown();

	private:
		WebNode _webNode;
	};
}
