#include "ResourceManager.h"
#include "Resource.h"

#include <iostream>

ResourceMap	ResourceManager::_resources;
const std::string ResourceManager::CAT_SYMBOL("|");

ResourceManager::ResourceManager()
{
}


ResourceManager::~ResourceManager()
{
}

Resource * ResourceManager::getResource(const std::string & fileName,
	Resource * (*reader)(const std::string &),
	const std::string & suffix) {
	Resource * rsrc = 0x0;
	const std::string key = fileName + CAT_SYMBOL + suffix;
	ResourceMap::iterator itr = _resources.find(key);
	if (itr != _resources.end()) {
		rsrc = itr->second;
	}
	else {
		rsrc = reader(fileName);
		if (rsrc == 0x0){
			return NULL;
		}
		_resources[key] = rsrc;
	}

	return rsrc;
}

void ResourceManager::cleanup() {
	ResourceMap::iterator itr = _resources.begin();
	while (itr != _resources.end()) {
		Resource * rsrc = itr->second;
		if (rsrc->isUnreferenced()) {
			rsrc->destroy();
			ResourceMap::iterator next = itr;
			++next;
			_resources.erase(itr);
			itr = next;
		}
		else {
			++itr;
		}
	}
}

bool ResourceManager::removeResource(Resource * rsrc)
{
	const std::string key = rsrc->_fileName + CAT_SYMBOL + rsrc->getLabel();
	ResourceMap::iterator itr = _resources.find(key);
	if (itr == _resources.end()) {
		return false;
	}
	if (!rsrc->isUnreferenced()) {
		return false;
	}
	_resources.erase(itr);
	rsrc->destroy();
	return true;
}