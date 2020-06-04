#pragma once


#ifdef FC_CLIENT_EXPORTS  
#define FC_CLIENT_API __declspec(dllexport)   
#else  
#define FC_CLIENT_API __declspec(dllimport)   
#endif
