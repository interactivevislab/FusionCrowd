#pragma once

#include <map>
#include <string>

#include "Config.h"

// Forward declaration
class Resource;

typedef std::map< std::string, Resource * > ResourceMap;

class FUSION_CROWD_API ResourceManager
{
public:
	ResourceManager();
	~ResourceManager();

	static Resource * getResource(const std::string & fileName,
		Resource * (*reader)(const std::string &),
		const std::string & suffix);

	static void cleanup();
	static bool removeResource(Resource * rsrc);

//protected:
	static ResourceMap	_resources;

//private:
	static const std::string CAT_SYMBOL;

};

