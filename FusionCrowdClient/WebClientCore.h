#pragma once

#include "FcClientApi.h"

#include "WebCore.h"


namespace FusionCrowdWeb
{
	class FC_CLIENT_API WebClientCore : public WebCore
	{
	public:
		WebClientCore();

		void Connect(const char* inIpAdress, short inPort);
	};
}