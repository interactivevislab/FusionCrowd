#pragma once

#include "WebCore.h"


namespace FusionCrowdWeb
{
	class WebServerCore : public WebCore
	{
	public:
		WebServerCore();

		void Bind(const char* inIpAdress, short inPort);
		void Listen();
		void Accept();
	};
}
