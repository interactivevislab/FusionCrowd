#include "stdafx.h"
#include "CppUnitTest.h"

#include <fstream>

#include "Export/FCArray.h"


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace FusionCrowd;

namespace UnitTest
{
	TEST_CLASS(FCArrayUnitTest)
	{
	public:
		TEST_METHOD(FCArray__Create_non_empty)
		{
			FCArray<size_t> arr(5);

			Assert::IsTrue(arr.size() == 5);
		}

		TEST_METHOD(FCArray__Create_empty)
		{
			FCArray<size_t> arr(0);

			Assert::IsTrue(arr.size() == 0);
		}

		TEST_METHOD(FCArray__Index_element)
		{
			FCArray<size_t> arr(4);

			// writes
			for(size_t i = 0; i < arr.size(); i++)
			{
				arr[i] = i;
			}

			// reads
			for(size_t i = 0; i < arr.size(); i++)
			{
				Assert::IsTrue(arr[i] == i);
			}

			auto f = [&]() { };

			try
			{
				arr[4] = 0;
				Assert::Fail(L"Must throw an exception, out of bounds");
			} catch(...)
			{
				Assert::IsTrue(true);
			}
		}

		TEST_METHOD(FCArray__Copy_non_empty)
		{
			FCArray<size_t> arr(5);
			for(size_t i = 0; i < arr.size(); i++)
			{
				arr[i] = i;
			}

			FCArray<size_t> copy = arr;

			Assert::IsTrue(copy.size() == 5);
			for(size_t i = 0; i < copy.size(); i++)
			{
				Assert::IsTrue(copy[i] == i);
			}
		}

		TEST_METHOD(FCArray__Copy_empty)
		{
			FCArray<size_t> arr(0);
			FCArray<size_t> copy = arr;

			Assert::IsTrue(copy.size() == 0);
		}

		TEST_METHOD(FCArray__Destructor_non_empty)
		{
			auto arr = new FCArray<size_t>(5);

			delete arr;
		}

		TEST_METHOD(FCArray__Destructor_empty)
		{
			auto arr = new FCArray<size_t>(0);

			delete arr;
		}

		TEST_METHOD(FCArray__Move_non_empty)
		{
			FCArray<size_t> arr(5);
			FCArray<size_t> other = std::move(arr);
		}

		TEST_METHOD(FCArray__Move_empty)
		{
			FCArray<size_t> arr(0);
			FCArray<size_t> other = std::move(arr);
		}
	};
}