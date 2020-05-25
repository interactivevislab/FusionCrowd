#pragma once


#ifdef FC_WEB_EXPORTS  
#define FC_WEB_API __declspec(dllexport)   
#else  
#define FC_WEB_API __declspec(dllimport)   
#endif
