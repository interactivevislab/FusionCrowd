#pragma once


#ifdef FC_MAIN_SERVER_EXPORTS  
#define FC_MAIN_SERVER_API __declspec(dllexport)   
#else  
#define FC_MAIN_SERVER_API __declspec(dllimport)   
#endif
