#include "pch.h"

#include "EIFPDDataset.h"

using namespace TestFusionCrowd;

EIFPDDataset::EIFPDDataset(std::string path)
{
}

EIFPDRecording EIFPDDataset::GetRecording()
{
	throw std::logic_error("Function not yet implemented");
}

EIFPDStrategy EIFPDDataset::GetStrategy()
{
	throw std::logic_error("Function not yet implemented");
}

EIFPDConfig EIFPDDataset::GetConfig()
{
	throw std::logic_error("Function not yet implemented");
}

FusionCrowd::NavSystem EIFPDDataset::GetNavSystem()
{
	throw std::logic_error("Function not yet implemented");
}


EIFPDDataset::~EIFPDDataset()
{
}
