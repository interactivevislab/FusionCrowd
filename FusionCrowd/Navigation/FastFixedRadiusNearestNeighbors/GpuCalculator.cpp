#include "GpuCalculator.h"
#include <cstring>

namespace FusionCrowd
{
	GpuCalculator::GpuCalculator()
	{
	}


	GpuCalculator::~GpuCalculator()
	{
#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if (p) { (p)->Release(); (p)=nullptr; } }
#endif
		for (int i = 0; i < _numberOfInputBuffers; i++) {
			SAFE_RELEASE(_inputBuffersSRV[i]);
			SAFE_RELEASE(_inputBuffers[i]);
		}

		delete[] _inputBuffersSRV;
		delete[] _inputBuffers;

		SAFE_RELEASE(_outputBufferUAV);
		SAFE_RELEASE(_outputBuffer);
		SAFE_RELEASE(_constantBuffer);
		SAFE_RELEASE(_shader);
		FreeUnusedMemory();
		SAFE_RELEASE(_context);
		SAFE_RELEASE(_device);
	}


	bool GpuCalculator::Init() {
		return !FAILED(GpuHelper::CreateComputeDevice(&_device, &_context, false));
	}


	bool GpuCalculator::LoadShader(LPCWSTR sourseFile, LPCSTR functionName, const D3D_SHADER_MACRO defines[]) {
		return !FAILED(GpuHelper::CreateComputeShader(sourseFile, functionName, defines, _device, &_shader));
	}


	void GpuCalculator::SetInputBuffers(int numberOfBuffers, InputBufferDesc descriptions[]) {
		for (int i = 0; i < _numberOfInputBuffers; i++) {
			_inputBuffers[i]->Release();
			_inputBuffersSRV[i]->Release();
		}
		delete[] _inputBuffers;
		delete[] _inputBuffersSRV;

		_numberOfInputBuffers = numberOfBuffers;
		_inputBuffers = new ID3D11Buffer*[numberOfBuffers];
		_inputBuffersSRV = new ID3D11ShaderResourceView*[numberOfBuffers];
		for (int i = 0; i < numberOfBuffers; i++) {
			GpuHelper::CreateStructuredBuffer(_device, descriptions[i].elementSize, descriptions[i].elementsCount, descriptions[i].initDataSource, 
				D3D11_CPU_ACCESS_WRITE, &_inputBuffers[i]);
			GpuHelper::CreateBufferSRV(_device, _inputBuffers[i], &_inputBuffersSRV[i]);
		}
	}


	void GpuCalculator::SetOutputBuffer(int elementSize, int elementsCount) {
		if (_outputBuffer != nullptr) {
			_outputBuffer->Release();
		}
		if (_outputBufferUAV != nullptr) {
			_outputBufferUAV->Release();
		}

		_outputElementsSize = elementSize;
		_outputElementsCount = elementsCount;
		GpuHelper::CreateStructuredBuffer(_device, elementSize, elementsCount, nullptr, D3D11_CPU_ACCESS_READ ,&_outputBuffer);
		GpuHelper::CreateBufferUAV(_device, _outputBuffer, &_outputBufferUAV);
	}


	void GpuCalculator::SetConstantBuffer(int elementSize, int elementsCount, void* initDataSource) {
		if (_constantBuffer != nullptr) {
			if (elementSize * elementsCount <= _constantElementsSize * _constantElementsCount) {
				GpuHelper::WriteDataToBuffer(_constantBuffer, initDataSource, elementSize * elementsCount, _context);
				return;
			}
			else 
			{
				_constantBuffer->Release();
			}
		}

		GpuHelper::CreateStructuredBuffer(_device, elementSize, elementsCount, initDataSource, D3D11_CPU_ACCESS_WRITE, &_constantBuffer);
		_context->CSSetConstantBuffers(0, 1, &_constantBuffer);
	}


	void GpuCalculator::RunShader() {
		GpuHelper::RunComputeShader(_context, _shader, _numberOfInputBuffers, _inputBuffersSRV, nullptr, nullptr, 0, _outputBufferUAV, _outputElementsCount, 1, 1);
	}


	void GpuCalculator::GetResult(void* outResult) {
		GpuHelper::ReadDataFromBuffer(_outputBuffer, outResult, _outputElementsSize * _outputElementsCount, _context);
	}


	void GpuCalculator::FreeUnusedMemory() {
		_context->ClearState();
		_context->Flush();
	}
}