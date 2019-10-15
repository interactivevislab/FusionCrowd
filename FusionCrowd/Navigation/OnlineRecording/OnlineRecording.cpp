#include "OnlineRecording.h"

#include <map>
#include <vector>
#include <algorithm>

#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class OnlineRecording::OnlineRecordingImpl
	{
	public:
		OnlineRecordingImpl()
		{
			m_currentSlice = OnlineRecordingSlice(0);
		}

		size_t GetSlicesCount() const {
			return m_slices.size();
		}

		void GetTimeSpan(TimeSpan & outTimeSpan) const
		{
			std::copy(m_snapshotTimes.begin(), m_snapshotTimes.end(), outTimeSpan.begin());
		}

		const OnlineRecordingSlice & GetSlice(float time) const
		{
			assert(time >= 0 && "Time must be non-negative");

			if(time >= m_currentTime)
				return m_currentSlice;

			auto result = std::lower_bound(m_snapshotTimes.begin(), m_snapshotTimes.end(), time);

			if(result == m_snapshotTimes.end())
				result = std::prev(m_snapshotTimes.end());

			size_t i = result - m_snapshotTimes.begin();

			return m_slices[i];
		}

		const OnlineRecordingSlice & GetCurrentSlice() const {
			return m_currentSlice;
		}

		const IRecordingSlice * Begin() const {
			return m_slices.begin()._Ptr;
		}

		const IRecordingSlice * End() const {
			return m_slices.end()._Ptr;
		}

		size_t GetAgentCount() const
		{
			return std::max(m_prevAgentCount, m_currentSlice.GetAgentCount());
		}

		AgentInfo GetAgentInfo(size_t agentId, float time)
		{
			return GetSlice(time).GetAgentInfo(agentId);
		}

		void MakeRecord(FCArray<AgentInfo> agentsInfos, float timeStep)
		{
			assert(timeStep > 0 && "Time step must be positive");

			m_snapshotTimes.push_back(m_currentTime);
			m_slices.push_back(m_currentSlice);
			m_prevAgentCount = std::max(m_prevAgentCount, m_currentSlice.GetAgentCount());

			m_currentTime += timeStep;
			m_currentSlice = OnlineRecordingSlice(std::move(agentsInfos), m_currentTime);
		}

		void GetAgentIds(FCArray<size_t> & outIds)
		{
			m_currentSlice.GetAgentIds(outIds);
		}

		~OnlineRecordingImpl() {}

	private:
		float m_currentTime = 0;
		OnlineRecordingSlice m_currentSlice = OnlineRecordingSlice(0);

		size_t m_prevAgentCount = 0;
		std::vector<float> m_snapshotTimes;
		//timeId -> agentId -> agentInfo
		std::vector<OnlineRecordingSlice> m_slices;
	};

	OnlineRecording::OnlineRecording()
		: pimpl(std::make_unique<OnlineRecordingImpl>())
	{
	}

	size_t OnlineRecording::GetSlicesCount() const {
		return pimpl->GetSlicesCount();
	}

	void OnlineRecording::GetTimeSpan(TimeSpan & outTimeSpan) const
	{
		pimpl->GetTimeSpan(outTimeSpan);
	}

	const OnlineRecordingSlice & OnlineRecording::GetSlice(float time) const
	{
		return pimpl->GetSlice(time);
	}

	const OnlineRecordingSlice & OnlineRecording::GetCurrentSlice() const
	{
		return pimpl->GetCurrentSlice();
	}

	const IRecordingSlice * OnlineRecording::Begin() const
	{
		return pimpl->Begin();
	}

	const IRecordingSlice * OnlineRecording::End() const
	{
		return pimpl->End();
	}

	size_t OnlineRecording::GetAgentCount() const
	{
		return pimpl->GetAgentCount();
	}

	void OnlineRecording::MakeRecord(FCArray<AgentInfo> agentsInfos, float timeStep)
	{
		pimpl->MakeRecord(std::move(agentsInfos), timeStep);
	}

	void OnlineRecording::GetAgentIds(FCArray<size_t> & outIds)
	{
		pimpl->GetAgentIds(outIds);
	}

	OnlineRecording::OnlineRecording(OnlineRecording && other) = default;
	OnlineRecording & OnlineRecording::operator=(OnlineRecording && other) = default;
	OnlineRecording::~OnlineRecording() = default;
}
