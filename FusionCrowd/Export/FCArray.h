#pragma once

#include "Export/Config.h"

namespace FusionCrowd
{
	template<typename T>
	struct FUSION_CROWD_API FCArray
	{
		FCArray(const size_t len);

		FCArray(FCArray &&);
		FCArray & operator=(FCArray &&);

		FCArray(const FCArray &);
		FCArray & operator =(const FCArray &);

		T & operator[](int index) const;
		size_t size() const;

		T* begin() const;
		T* end() const;

		~FCArray();
	private:
		T* vals;
		size_t len;
	};

	template<typename T>
	inline FCArray<T>::FCArray(const size_t l)
		: vals(new T[l + 1]), len(l)
	{
	}

	template<typename T>
	inline FCArray<T>::FCArray(FCArray<T> && other)
		: vals(other.vals), len(other.len)
	{
		other.vals = nullptr;
		other.len = 0;
	}

	template<typename T>
	inline FCArray<T> & FCArray<T>::operator=(FCArray<T> && other)
	{
		if(this != &other)
		{
			this.vals = other.vals;
			this.len = other.len;

			other.vals = nullptr;
			other.len = 0;
		}

		return *this;
	}

	template<typename T>
	inline FCArray<T>::FCArray(const FCArray<T> & other)
		: vals(new T[other.len + 1]), len(other.len)
	{
		for(size_t i = 0; i < len; i++)
		{
			vals[i] = other.vals[i];
		}
	}

	template<typename T>
	inline FCArray<T> & FCArray<T>::operator=(const FCArray<T> & other)
	{
		if(this != &other)
		{
			if(this.vals != nullptr)
				delete[] this.vals;

			this.vals = new T[other.len + 1];
			this.len = other.len;

			for(size_t i = 0; i < len; i++)
			{
				vals[i] = other.vals[i];
			}
		}

		return *this;
	}

	template<typename T>
	inline T& FCArray<T>::operator[](int index) const
	{
		if (index >= len || index < 0)
			throw "Index is out of range";

		return vals[index];
	}

	template<typename T>
	inline T* FCArray<T>::begin() const { return vals; }

	template<typename T>
	inline T* FCArray<T>::end() const { return &vals[len]; }

	template<typename T>
	inline size_t FCArray<T>::size() const { return len; }

	template<typename T>
	inline FCArray<T>::~FCArray()
	{
		if(vals != nullptr)
		{
			delete[] vals;
		}
	}

	template<typename T>
	void FCArrayDeallocate(FCArray<T> * array)
	{
		delete array;
	}
}
