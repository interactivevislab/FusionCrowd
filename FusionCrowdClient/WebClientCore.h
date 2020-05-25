#pragma once

#include "FcClientApi.h"

#include "WebCore.h"


namespace FusionCrowdWeb
{
	class WebClientCore : public WebCore
	{
	public:
		WebClientCore();

		void Connect(const char* inIpAdress, short inPort);
	};
}