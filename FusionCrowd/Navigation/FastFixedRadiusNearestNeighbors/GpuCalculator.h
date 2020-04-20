#pragma once

#include "GpuHelper.h"

namespace FusionCrowd
{
	struct InputBufferDesc {
		int elementSize;
		int elementsCount;
		void* initDataSource;
	};

	class GpuCalculator
	{
	private:
		ID3D11Device*               _device = nullptr;
		ID3D11DeviceContext*        _context = nullptr;
		ID3D11ComputeShader*        _shader = nullptr;

		ID3D11Buffer**              _inputBuffers = nullptr;
		ID3D11Buffer*               _outputBuffer = nullptr;
		ID3D11Buffer*               _constantBuffer = nullptr;

		ID3D11ShaderResourceView**  _inputBuffersSRV = nullptr;
		ID3D11UnorderedAccessView*  _outputBufferUAV = nullptr;

		int	_numberOfInputBuffers = 0;
		int _outputElementsCount = 0;
		int _outputElementsSize = 0;
		int _constantElementsSize = 0;
		int _constantElementsCount = 0;

	public:
		GpuCalculator();
		~GpuCalculator();

		bool Init(); //CreatingDevice + CreatingContext
		bool LoadShader(LPCWSTR sourseFile, LPCSTR functionName, const D3D_SHADER_MACRO defines[]);
		void SetInputBuffers(int numberOfBuffers, InputBufferDesc descriptions[]); //+ CreatingViews
		void SetOutputBuffer(int elementSize, int elementsCount); //+ CreatingView
		void SetConstantBuffer(int elementSize, int elementsCount, void* initDataSource);
		void RunShader();
		void GetResult(void* outResult);
		void FreeUnusedMemory();
	};

}