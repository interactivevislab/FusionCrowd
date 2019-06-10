#pragma once

#include "GpuHelper.h"
#include "Config.h"

namespace FusionCrowd
{

	struct FUSION_CROWD_API InputBufferDesc {
		int elementSize;
		int elementsCount;
		void* initDataSource;
	};

	class FUSION_CROWD_API GpuCalculator
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

		int	_numberOfInputBuffers;
		int _outputElementsCount;
		int _outputElementsSize;
		int _constantElementsSize;
		int _constantElementsCount;

	public:
		GpuCalculator();
		~GpuCalculator();

		bool Init(); //CreatingDevice + CreatingContext
		bool LoadShader(LPCWSTR sourseFile, LPCSTR functionName);
		void SetInputBuffers(int numberOfBuffers, InputBufferDesc descriptions[]); //+ CreatingViews
		void SetOutputBuffer(int elementSize, int elementsCount); //+ CreatingView
		void SetConstantBuffer(int elementSize, int elementsCount, void* initDataSource);
		void RunShader();
		void* GetResult();
		void FreeUnusedMemory();
	};

}