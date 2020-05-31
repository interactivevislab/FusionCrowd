#pragma once

#include "FcMainServerApi.h"
#include "WebNode.h"


namespace FusionCrowdWeb
{
	class FC_MAIN_SERVER_API FcMainServer
	{
	public:
		void StartOn(const char* inIpAdress, short inPort);
		void Shutdown();

	private:
		WebNode _webNode;
	};
}
