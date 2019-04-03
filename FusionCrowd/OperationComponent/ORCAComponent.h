#pragma once
#include "../IOperComponent.h"
#include "../Agent.h"
#include "../Config.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class FUSION_CROWD_API ORCAComponent :
			public IOperComponent
		{
		public:
			ORCAComponent();
			ORCAComponent(float timeHorizon, float timeHorizonObst);
			void Update(float timeStep);
			~ORCAComponent();

		private:
			float _timeHorizon;
			float _timeHorizonObst;

		};
	}
}
