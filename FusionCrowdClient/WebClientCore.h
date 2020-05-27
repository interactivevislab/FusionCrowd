#pragma once

#include "FcClientApi.h"

#include "WebCore.h"


namespace FusionCrowdWeb
{
	class WebClientCore : public WebCore
	{
	public:
		void Connect(const char* inIpAdress, short inPort);
		void Disconnect();

		void Send(WebCode inWebCode, const char* inData, size_t inDataSize);
		std::pair <WebCode, const char*> Receive();
	};
}