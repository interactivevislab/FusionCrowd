#include "pch.h"

#include "EIFPDRecording.h"

#include <stdexcept>

using namespace TestFusionCrowd;


/*
EIFPDRecording EIFPDRecording::Read(char * filepath)
{
	return EIFPDRecording();
}
*/

FusionCrowd::TimeSpan EIFPDRecording::GetTimeSpan() const
{
	return FusionCrowd::TimeSpan(1);
}


const FusionCrowd::IRecordingSlice & EIFPDRecording::GetSlice(float time) const
{
	throw std::logic_error("Function not yet implemented");
}


EIFPDRecording::EIFPDRecording()
{
}

EIFPDRecording::~EIFPDRecording()
{
}
