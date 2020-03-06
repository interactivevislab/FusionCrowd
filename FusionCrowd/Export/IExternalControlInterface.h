#pragma once

namespace FusionCrowd
{
	class FUSION_CROWD_API IExternalControllInterface
	{
	public:
		virtual bool AddInput(size_t agent_id, float x, float y) = 0;
	};
}