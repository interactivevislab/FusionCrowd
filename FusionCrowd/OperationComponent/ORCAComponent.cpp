#include "ORCAComponent.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		ORCAComponent::ORCAComponent() : _timeHorizon(2.5f), _timeHorizonObst(0.15f)
		{

		}

		ORCAComponent::ORCAComponent(float timeHorizon, float timeHorizonObst) : _timeHorizon(timeHorizon), _timeHorizonObst(timeHorizonObst)
		{
		}

		void ORCAComponent::Update(float timeStep)
		{

		}


		ORCAComponent::~ORCAComponent()
		{
		}
	}
}
