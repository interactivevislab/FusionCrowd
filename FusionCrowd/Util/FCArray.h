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

		T & operator[](int index);

		T* begin();
		T* end();

		T* const vals;
		const size_t len;

		~FCArray();
	};

	template<typename T>
	inline FCArray<T>::FCArray(const size_t l)
		: vals(new T[l + 1]), len(l)
	{
	}

	template<typename T>
	inline FCArray<T>::FCArray(FCArray && other)
		: vals(other.vals), len(other.len)
	{
	}

	template<typename T>
	inline FCArray<T> & FCArray<T>::operator=(FCArray<T> && other)
	{
		if(this != &other)
		{
			std::move(other.vals, this.vals);
			std::move(other.len, this.len);
		}

		return *this;
	}

	template<typename T>
	inline T& FCArray<T>::operator[](int index)
	{
		if(index >= len || -index >= len)
			throw "Index is out of range";

		return (index >= 0) ? vals[index] : vals[len + index - 1];
	}

	template<typename T>
	inline T* FCArray<T>::begin() { return vals; }

	template<typename T>
	inline T* FCArray<T>::end() { return &vals[len]; }

	template<typename T>
	inline FCArray<T>::~FCArray()
	{
		delete[] vals;
	}
}
