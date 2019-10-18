#include "GpuHelper.h"
#include <stdio.h>
#include <crtdbg.h>
#include <d3dcommon.h>
#include <d3d11.h>
#include <d3dcompiler.h>

#include "NeighborsSeekerShader.h"

namespace FusionCrowd
{

	namespace GpuHelper {

#if D3D_COMPILER_VERSION < 46
#include <d3dx11.h>
#endif

#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if (p) { (p)->Release(); (p)=nullptr; } }
#endif

		//--------------------------------------------------------------------------------------
		// Create the D3D device and device context suitable for running Compute Shaders(CS)
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateComputeDevice(ID3D11Device** ppDeviceOut, ID3D11DeviceContext** ppContextOut, bool bForceRef)
		{
			*ppDeviceOut = nullptr;
			*ppContextOut = nullptr;

			HRESULT hr = S_OK;

			UINT uCreationFlags = D3D11_CREATE_DEVICE_SINGLETHREADED;
#ifdef _DEBUG
			uCreationFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
			D3D_FEATURE_LEVEL flOut;
			static const D3D_FEATURE_LEVEL flvl[] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0 };

			bool bNeedRefDevice = false;
			if (!bForceRef)
			{
				hr = D3D11CreateDevice(nullptr,                        // Use default graphics card
					D3D_DRIVER_TYPE_HARDWARE,    // Try to create a hardware accelerated device
					nullptr,                        // Do not use external software rasterizer module
					uCreationFlags,              // Device creation flags
					flvl,
					sizeof(flvl) / sizeof(D3D_FEATURE_LEVEL),
					D3D11_SDK_VERSION,           // SDK version
					ppDeviceOut,                 // Device out
					&flOut,                      // Actual feature level created
					ppContextOut);              // Context out

				if (SUCCEEDED(hr))
				{
					// A hardware accelerated device has been created, so check for Compute Shader support

					// If we have a device >= D3D_FEATURE_LEVEL_11_0 created, full CS5.0 support is guaranteed, no need for further checks
					if (flOut < D3D_FEATURE_LEVEL_11_0)
					{
#ifdef TEST_DOUBLE
						bNeedRefDevice = true;
						printf("No hardware Compute Shader 5.0 capable device found (required for doubles), trying to create ref device.\n");
#else
						// Otherwise, we need further check whether this device support CS4.x (Compute on 10)
						D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS hwopts;
						(*ppDeviceOut)->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &hwopts, sizeof(hwopts));
						if (!hwopts.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x)
						{
							bNeedRefDevice = true;
							printf("No hardware Compute Shader capable device found, trying to create ref device.\n");
						}
#endif
					}

#ifdef TEST_DOUBLE
					else
					{
						// Double-precision support is an optional feature of CS 5.0
						D3D11_FEATURE_DATA_DOUBLES hwopts;
						(*ppDeviceOut)->CheckFeatureSupport(D3D11_FEATURE_DOUBLES, &hwopts, sizeof(hwopts));
						if (!hwopts.DoublePrecisionFloatShaderOps)
						{
							bNeedRefDevice = true;
							printf("No hardware double-precision capable device found, trying to create ref device.\n");
						}
					}
#endif
				}
			}

			if (bForceRef || FAILED(hr) || bNeedRefDevice)
			{
				// Either because of failure on creating a hardware device or hardware lacking CS capability, we create a ref device here

				SAFE_RELEASE(*ppDeviceOut);
				SAFE_RELEASE(*ppContextOut);

				hr = D3D11CreateDevice(nullptr,                        // Use default graphics card
					D3D_DRIVER_TYPE_REFERENCE,   // Try to create a hardware accelerated device
					nullptr,                        // Do not use external software rasterizer module
					uCreationFlags,              // Device creation flags
					flvl,
					sizeof(flvl) / sizeof(D3D_FEATURE_LEVEL),
					D3D11_SDK_VERSION,           // SDK version
					ppDeviceOut,                 // Device out
					&flOut,                      // Actual feature level created
					ppContextOut);              // Context out
				if (FAILED(hr))
				{
					printf("Reference rasterizer device create failure\n");
					return hr;
				}
			}

			return hr;
		}

		//--------------------------------------------------------------------------------------
		// Compile and create the CS
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateComputeShader(LPCWSTR pSrcFile, LPCSTR pFunctionName, const D3D_SHADER_MACRO defines[],
				ID3D11Device* pDevice, ID3D11ComputeShader** ppShaderOut)
		{
			if (!pDevice || !ppShaderOut)
				return E_INVALIDARG;

			// Finds the correct path for the shader file.
			// This is only required for this sample to be run correctly from within the Sample Browser,
			// in your own projects, these lines could be removed safely
			/*
			WCHAR str[MAX_PATH];
			HRESULT hr = FindDXSDKShaderFileCch(str, MAX_PATH, pSrcFile);
			if (FAILED(hr))
				return hr;
			*/

			DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
			// Set the D3DCOMPILE_DEBUG flag to embed debug information in the shaders.
			// Setting this flag improves the shader debugging experience, but still allows
			// the shaders to be optimized and to run exactly the way they will run in
			// the release configuration of this program.
			dwShaderFlags |= D3DCOMPILE_DEBUG;

			// Disable optimizations to further improve shader debugging
			dwShaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

			// We generally prefer to use the higher CS shader profile when possible as CS 5.0 is better performance on 11-class hardware
			LPCSTR pProfile = (pDevice->GetFeatureLevel() >= D3D_FEATURE_LEVEL_11_0) ? "cs_5_0" : "cs_4_0";

			ID3DBlob* pErrorBlob = nullptr;
			ID3DBlob* pBlob = nullptr;

#if D3D_COMPILER_VERSION >= 46

			HRESULT hr = D3DCompile(
				(LPCSTR) NEIGHBORS_SEEKER_COMPUTE_SHADER, strlen(NEIGHBORS_SEEKER_COMPUTE_SHADER), "NeighboursSeeker",
				defines, D3D_COMPILE_STANDARD_FILE_INCLUDE, pFunctionName, pProfile,
				dwShaderFlags, 0, &pBlob, &pErrorBlob
			);
#else

			hr = D3DX11CompileFromFile(str, defines, nullptr, pFunctionName, pProfile,
				dwShaderFlags, 0, nullptr, &pBlob, &pErrorBlob, nullptr);

#endif

			if (FAILED(hr))
			{
				if (pErrorBlob)
					OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());

				SAFE_RELEASE(pErrorBlob);
				SAFE_RELEASE(pBlob);

				return hr;
			}

			hr = pDevice->CreateComputeShader(pBlob->GetBufferPointer(), pBlob->GetBufferSize(), nullptr, ppShaderOut);

			SAFE_RELEASE(pErrorBlob);
			SAFE_RELEASE(pBlob);

			return hr;
		}

		//--------------------------------------------------------------------------------------
		// Create Structured Buffer
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateStructuredBuffer(ID3D11Device* pDevice, UINT uElementSize, UINT uCount, void* pInitData,
				D3D11_CPU_ACCESS_FLAG cpuAccessFlag, D3D11_USAGE usageFlag, ID3D11Buffer** ppBufOut)
		{
			*ppBufOut = nullptr;

			D3D11_BUFFER_DESC desc;
			ZeroMemory(&desc, sizeof(desc));
			desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
			desc.ByteWidth = uElementSize * uCount;
			desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
			desc.StructureByteStride = uElementSize;
			desc.CPUAccessFlags = cpuAccessFlag;
			//desc.Usage = usageFlag;

			if (pInitData)
			{
				D3D11_SUBRESOURCE_DATA InitData;
				InitData.pSysMem = pInitData;
				return pDevice->CreateBuffer(&desc, &InitData, ppBufOut);
			}
			else
				return pDevice->CreateBuffer(&desc, nullptr, ppBufOut);
		}

		//--------------------------------------------------------------------------------------
		// Create Raw Buffer
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateRawBuffer(ID3D11Device* pDevice, UINT uSize, void* pInitData, ID3D11Buffer** ppBufOut)
		{
			*ppBufOut = nullptr;

			D3D11_BUFFER_DESC desc;
			ZeroMemory(&desc, sizeof(desc));
			desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_INDEX_BUFFER | D3D11_BIND_VERTEX_BUFFER;
			desc.ByteWidth = uSize;
			desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;

			if (pInitData)
			{
				D3D11_SUBRESOURCE_DATA InitData;
				InitData.pSysMem = pInitData;
				return pDevice->CreateBuffer(&desc, &InitData, ppBufOut);
			}
			else
				return pDevice->CreateBuffer(&desc, nullptr, ppBufOut);
		}

		//--------------------------------------------------------------------------------------
		// Create Shader Resource View for Structured or Raw Buffers
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateBufferSRV(ID3D11Device* pDevice, ID3D11Buffer* pBuffer, ID3D11ShaderResourceView** ppSRVOut)
		{
			D3D11_BUFFER_DESC descBuf;
			ZeroMemory(&descBuf, sizeof(descBuf));
			pBuffer->GetDesc(&descBuf);

			D3D11_SHADER_RESOURCE_VIEW_DESC desc;
			ZeroMemory(&desc, sizeof(desc));
			desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
			desc.BufferEx.FirstElement = 0;

			if (descBuf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS)
			{
				// This is a Raw Buffer

				desc.Format = DXGI_FORMAT_R32_TYPELESS;
				desc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
				desc.BufferEx.NumElements = descBuf.ByteWidth / 4;
			}
			else
				if (descBuf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_STRUCTURED)
				{
					// This is a Structured Buffer

					desc.Format = DXGI_FORMAT_UNKNOWN;
					desc.BufferEx.NumElements = descBuf.ByteWidth / descBuf.StructureByteStride;
				}
				else
				{
					return E_INVALIDARG;
				}

			return pDevice->CreateShaderResourceView(pBuffer, &desc, ppSRVOut);
		}

		//--------------------------------------------------------------------------------------
		// Create Unordered Access View for Structured or Raw Buffers
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT CreateBufferUAV(ID3D11Device* pDevice, ID3D11Buffer* pBuffer, ID3D11UnorderedAccessView** ppUAVOut)
		{
			D3D11_BUFFER_DESC descBuf;
			ZeroMemory(&descBuf, sizeof(descBuf));
			pBuffer->GetDesc(&descBuf);

			D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
			ZeroMemory(&desc, sizeof(desc));
			desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
			desc.Buffer.FirstElement = 0;

			if (descBuf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS)
			{
				// This is a Raw Buffer

				desc.Format = DXGI_FORMAT_R32_TYPELESS; // Format must be DXGI_FORMAT_R32_TYPELESS, when creating Raw Unordered Access View
				desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
				desc.Buffer.NumElements = descBuf.ByteWidth / 4;
			}
			else
				if (descBuf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_STRUCTURED)
				{
					// This is a Structured Buffer

					desc.Format = DXGI_FORMAT_UNKNOWN;      // Format must be must be DXGI_FORMAT_UNKNOWN, when creating a View of a Structured Buffer
					desc.Buffer.NumElements = descBuf.ByteWidth / descBuf.StructureByteStride;
				}
				else
				{
					return E_INVALIDARG;
				}

			return pDevice->CreateUnorderedAccessView(pBuffer, &desc, ppUAVOut);
		}

		void WriteDataToBuffer(ID3D11Buffer* buffer, void* dataSource, size_t dataSize, ID3D11DeviceContext* context) {
			D3D11_MAPPED_SUBRESOURCE mappedResource;
			context->Map(buffer, 0, D3D11_MAP_WRITE, 0, &mappedResource);
			memcpy(mappedResource.pData, dataSource, dataSize);
			context->Unmap(buffer, 0);
		}

		void ReadDataFromBuffer(ID3D11Buffer* buffer, void* dataDest, size_t dataSize, ID3D11DeviceContext* context) {
			D3D11_MAPPED_SUBRESOURCE mappedResource;
			context->Map(buffer, 0, D3D11_MAP_READ, 0, &mappedResource);
			memcpy(dataDest, mappedResource.pData, dataSize);
			context->Unmap(buffer, 0);
		}

		//--------------------------------------------------------------------------------------
		// Create a CPU accessible buffer and download the content of a GPU buffer into it
		// This function is very useful for debugging CS programs
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			ID3D11Buffer* CreateAndCopyToBuffer(ID3D11Device* pDevice, ID3D11DeviceContext* pd3dImmediateContext, ID3D11Buffer* pBuffer)
		{
			ID3D11Buffer* buffer = nullptr;

			D3D11_BUFFER_DESC desc;
			ZeroMemory(&desc, sizeof(desc));
			pBuffer->GetDesc(&desc);
			desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			desc.Usage = D3D11_USAGE_STAGING;
			desc.BindFlags = 0;
			desc.MiscFlags = 0;
			if (SUCCEEDED(pDevice->CreateBuffer(&desc, nullptr, &buffer)))
			{
				pd3dImmediateContext->CopyResource(buffer, pBuffer);
			}

			return buffer;
		}

		//--------------------------------------------------------------------------------------
		// Run CS
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			void RunComputeShader(ID3D11DeviceContext* pd3dImmediateContext,
				ID3D11ComputeShader* pComputeShader,
				UINT nNumViews, ID3D11ShaderResourceView** pShaderResourceViews,
				ID3D11Buffer* pCBCS, void* pCSData, DWORD dwNumDataBytes,
				ID3D11UnorderedAccessView* pUnorderedAccessView,
				UINT X, UINT Y, UINT Z)
		{
			pd3dImmediateContext->CSSetShader(pComputeShader, nullptr, 0);
			pd3dImmediateContext->CSSetShaderResources(0, nNumViews, pShaderResourceViews);
			pd3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &pUnorderedAccessView, nullptr);
			if (pCBCS && pCSData)
			{
				D3D11_MAPPED_SUBRESOURCE MappedResource;
				pd3dImmediateContext->Map(pCBCS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
				memcpy(MappedResource.pData, pCSData, dwNumDataBytes);
				pd3dImmediateContext->Unmap(pCBCS, 0);
				ID3D11Buffer* ppCB[1] = { pCBCS };
				pd3dImmediateContext->CSSetConstantBuffers(0, 1, ppCB);
			}

			pd3dImmediateContext->Dispatch(X, Y, Z);

			pd3dImmediateContext->CSSetShader(nullptr, nullptr, 0);

			ID3D11UnorderedAccessView* ppUAViewnullptr[1] = { nullptr };
			pd3dImmediateContext->CSSetUnorderedAccessViews(0, 1, ppUAViewnullptr, nullptr);

			ID3D11ShaderResourceView* ppSRVnullptr[2] = { nullptr, nullptr };
			pd3dImmediateContext->CSSetShaderResources(0, 2, ppSRVnullptr);

			ID3D11Buffer* ppCBnullptr[1] = { nullptr };
			pd3dImmediateContext->CSSetConstantBuffers(0, 1, ppCBnullptr);
		}

		//--------------------------------------------------------------------------------------
		// Tries to find the location of the shader file
		// This is a trimmed down version of DXUTFindDXSDKMediaFileCch.
		//--------------------------------------------------------------------------------------
		_Use_decl_annotations_
			HRESULT FindDXSDKShaderFileCch(WCHAR* strDestPath,
				int cchDest,
				LPCWSTR strFilename)
		{
			if (!strFilename || strFilename[0] == 0 || !strDestPath || cchDest < 10)
				return E_INVALIDARG;

			// Get the exe name, and exe path
			WCHAR strExePath[MAX_PATH] =
			{
				0
			};
			WCHAR strExeName[MAX_PATH] =
			{
				0
			};
			WCHAR* strLastSlash = nullptr;
			GetModuleFileName(nullptr, strExePath, MAX_PATH);
			strExePath[MAX_PATH - 1] = 0;
			strLastSlash = wcsrchr(strExePath, TEXT('\\'));
			if (strLastSlash)
			{
				wcscpy_s(strExeName, MAX_PATH, &strLastSlash[1]);

				// Chop the exe name from the exe path
				*strLastSlash = 0;

				// Chop the .exe from the exe name
				strLastSlash = wcsrchr(strExeName, TEXT('.'));
				if (strLastSlash)
					*strLastSlash = 0;
			}

			// Search in directories:
			//      .\
		    //      %EXE_DIR%\..\..\%EXE_NAME%

			wcscpy_s(strDestPath, cchDest, strFilename);
			if (GetFileAttributes(strDestPath) != 0xFFFFFFFF)
				return S_OK;

			swprintf_s(strDestPath, cchDest, L"%s\\..\\..\\%s\\%s", strExePath, strExeName, strFilename);
			if (GetFileAttributes(strDestPath) != 0xFFFFFFFF)
				return S_OK;

			// On failure, return the file as the path but also return an error code
			wcscpy_s(strDestPath, cchDest, strFilename);

			return E_FAIL;
		}

	}

}