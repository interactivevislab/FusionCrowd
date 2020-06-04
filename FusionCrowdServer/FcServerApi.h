#pragma once


#ifdef FC_SERVER_EXPORTS  
#define FC_SERVER_API __declspec(dllexport)   
#else  
#define FC_SERVER_API __declspec(dllimport)   
#endif
