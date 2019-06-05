#pragma once

#include <d3d11.h>

#if defined(_MSC_VER) && (_MSC_VER<1610) && !defined(_In_reads_)
#define _Outptr_
#define _Outptr_opt_ 
#define _In_reads_(exp)
#define _In_reads_opt_(exp)
#define _Out_writes_(exp)
#endif

#ifndef _Use_decl_annotations_
#define _Use_decl_annotations_
#endif

namespace GpuHelper
{
	HRESULT CreateComputeDevice(_Outptr_ ID3D11Device** ppDeviceOut, _Outptr_ ID3D11DeviceContext** ppContextOut, _In_ bool bForceRef);
	HRESULT CreateComputeShader(_In_z_ LPCWSTR pSrcFile, _In_z_ LPCSTR pFunctionName,
		_In_ ID3D11Device* pDevice, _Outptr_ ID3D11ComputeShader** ppShaderOut);
	HRESULT CreateStructuredBuffer(_In_ ID3D11Device* pDevice, _In_ UINT uElementSize, _In_ UINT uCount,
		_In_reads_(uElementSize*uCount) void* pInitData,
		_Outptr_ ID3D11Buffer** ppBufOut);
	HRESULT CreateRawBuffer(_In_ ID3D11Device* pDevice, _In_ UINT uSize, _In_reads_(uSize) void* pInitData, _Outptr_ ID3D11Buffer** ppBufOut);
	HRESULT CreateBufferSRV(_In_ ID3D11Device* pDevice, _In_ ID3D11Buffer* pBuffer, _Outptr_ ID3D11ShaderResourceView** ppSRVOut);
	HRESULT CreateBufferUAV(_In_ ID3D11Device* pDevice, _In_ ID3D11Buffer* pBuffer, _Outptr_ ID3D11UnorderedAccessView** pUAVOut);
	ID3D11Buffer* CreateAndCopyToBuffer(_In_ ID3D11Device* pDevice, _In_ ID3D11DeviceContext* pd3dImmediateContext, _In_ ID3D11Buffer* pBuffer);
	void RunComputeShader(_In_ ID3D11DeviceContext* pd3dImmediateContext,
		_In_ ID3D11ComputeShader* pComputeShader,
		_In_ UINT nNumViews, _In_reads_(nNumViews) ID3D11ShaderResourceView** pShaderResourceViews,
		_In_opt_ ID3D11Buffer* pCBCS, _In_reads_opt_(dwNumDataBytes) void* pCSData, _In_ DWORD dwNumDataBytes,
		_In_ ID3D11UnorderedAccessView* pUnorderedAccessView,
		_In_ UINT X, _In_ UINT Y, _In_ UINT Z);
	HRESULT FindDXSDKShaderFileCch(_Out_writes_(cchDest) WCHAR* strDestPath,
		_In_ int cchDest,
		_In_z_ LPCWSTR strFilename);
}

