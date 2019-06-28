#pragma once

#include "Config.h"

namespace FusionCrowd
{
	template<typename T>
	struct FUSION_CROWD_API FCArray
	{
		FCArray(const size_t len);

		FCArray(FCArray && other);

		FCArray & operator=(FCArray && other);

		T* begin();
		T* end();

		T* const vals;
		const size_t len;

		~FCArray();
	};

	template<typename T>
	FCArray<T>::FCArray(const size_t l)
		: vals(new T[len + 1]), len(l)
	{
		vals[len] = 0;
	}

	template<typename T>
	FCArray<T>::FCArray(FCArray && other)
		: vals(other.vals), len(other.len)
	{
	}

	template<typename T>
	FCArray<T> & FCArray<T>::operator=(FCArray<T> && other)
	{
		if(this != &other)
		{
			std::move(other.vals, this.vals);
			std::move(other.len, this.len);
		}

		return *this;
	}

	template<typename T>
	T* FCArray<T>::begin() { return vals; }

	template<typename T>
	T* FCArray<T>::end() { return &vals[len]; }

	template<typename T>
	FCArray<T>::~FCArray()
	{
		delete[] vals;
	}
}
