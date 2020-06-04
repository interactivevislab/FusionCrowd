#pragma once

#include "FusionCrowdWebApi.h"

#include <stdexcept>
#include <string.h>


namespace FusionCrowdWeb
{
	class FC_WEB_API FcWebException
	{
	public:

		FcWebException() noexcept;
		FcWebException(char const* const _Message) noexcept;
		FcWebException(FcWebException const& _Other) noexcept;

		virtual ~FcWebException() noexcept;

		FcWebException& operator=(FcWebException const& _Other) noexcept;

		virtual char const* What() const;

	private:
		char* _message = nullptr;
	};


	class FC_WEB_API WsException : public FcWebException
	{
	public:
		WsException(const char* inMessage);
		int GetWsErrorCode() const;

	private:
		int _errorCode;
	};
}
