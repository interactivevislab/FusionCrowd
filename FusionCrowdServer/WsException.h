#pragma once

#include <stdexcept>
#include<WinSock2.h>
#include <cstring>
#include <string.h>


namespace FusionCrowdWeb {

	struct WsException : std::runtime_error
	{
	private:
		int _errorCode;

	public:
		WsException(const char* message) : std::runtime_error(message) {
			_errorCode = WSAGetLastError();
		}

		int GetWSErrorCode() {
			return _errorCode;
		}
	};

}