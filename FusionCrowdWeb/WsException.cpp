#include "WsException.h"

#include <WinSock2.h>


namespace FusionCrowdWeb
{
	FcWebException::FcWebException() noexcept
	{
	}


	FcWebException::FcWebException(char const* const inMessage) noexcept
	{
		auto len = strlen(inMessage) + 1;
		_message = new char[len];
		strcpy_s(_message, len, inMessage);
	}


	FcWebException::FcWebException(FcWebException const& inOther) noexcept
	{
		auto len = strlen(inOther._message) + 1;
		_message = new char[len];
		strcpy_s(_message, len, inOther._message);
	}


	FcWebException& FcWebException::operator=(FcWebException const& inOther) noexcept
	{
		if (this == &inOther)
		{
			return *this;
		}

		if (_message)
		{
			delete _message;
		}
		auto len = strlen(inOther._message) + 1;
		_message = new char[len];
		strcpy_s(_message, len, inOther._message);

		return *this;
	}


	FcWebException::~FcWebException() noexcept
	{
		if (_message)
		{
			delete _message;
		}
	}


	char const* FcWebException::What() const
	{
		return _message ? _message : "Unknown exception";
	}


	WsException::WsException(const char* inMessage) : FcWebException(inMessage)
	{
		_errorCode = WSAGetLastError();
	}


	int WsException::GetWsErrorCode() const
	{
		return _errorCode;
	}
}
