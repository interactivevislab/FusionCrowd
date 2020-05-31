#pragma once


#ifdef FC_COMPUTATIONAL_SERVER_EXPORTS  
#define FC_COMPUTATIONAL_SERVER_API __declspec(dllexport)   
#else  
#define FC_COMPUTATIONAL_SERVER_API __declspec(dllimport)   
#endif
