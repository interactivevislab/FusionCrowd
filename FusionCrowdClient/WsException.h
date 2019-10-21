#pragma once

#include <stdexcept>
#include<WinSock2.h>
#include <cstring>
#include <string.h>

using RuntimeError = std::runtime_error;


struct WsException : RuntimeError
{
private:
	int _errorCode;

public:
	WsException(const char* message) : RuntimeError(message) {
		_errorCode = WSAGetLastError();
	}

	int GetWSErrorCode() {
		return _errorCode;
	}
};