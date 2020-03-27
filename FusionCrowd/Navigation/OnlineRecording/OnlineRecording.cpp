#include "OnlineRecording.h"

#include <map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

#include "Util/RecordingSerializer.h"
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

		bool LoadFromFile(char const * path, size_t path_length) {
			std::string filename(path, path_length);
			std::ifstream inFile(filename);
			if (!inFile.good()) return false;
			m_slices.clear();
			m_snapshotTimes.clear();
			std::string line;
			while (std::getline(inFile, line)) {
				int i = 0;
				std::stringstream ss(line);
				std::string value;
				bool first_value = true;
				OnlineRecordingSlice* cur_slice = nullptr;
				AgentInfo* cur_info = new AgentInfo();
				while (std::getline(ss, value, ',')) {
					if (first_value) {
						float slice_time = std::stof(value);
						cur_slice = new OnlineRecordingSlice(slice_time);
						m_snapshotTimes.push_back(slice_time);
						first_value = false;
						continue;
					}
					switch (i % 6) {
					case 0 :
						cur_info->id = std::stoi(value);
						break;
					case 1 :
						cur_info->posX = std::stof(value);
						break;
					case 2 :
						cur_info->posY = std::stof(value);
						break;
					case 3 :
						cur_info->orientX = std::stof(value);
						break;
					case 4 :
						cur_info->orientY = std::stof(value);
						break;
					case 5 :
						cur_info->radius = std::stof(value);
						break;
					}
					i++;
					if (i == 6) {
						i = 0;
						cur_slice->AddAgent(*cur_info);
						delete cur_info;
						cur_info = new AgentInfo();
					}
				}
				m_slices.push_back(*cur_slice);
				delete cur_slice;
				delete cur_info;
			}
			m_currentSlice = *std::prev(m_slices.end());
			m_currentTime = m_currentSlice.GetTime();
			m_prevAgentCount = std::prev(std::prev(m_slices.end()))->GetAgentCount();
			return true;
		}

		void Serialize(IRecording const &  rec, char const * destFilePath, size_t pathLen) const {
			FusionCrowd::Recordings::Serialize(rec, destFilePath, pathLen);
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

	bool OnlineRecording::LoadFromFile(char const * path, size_t path_length) {
		return pimpl->LoadFromFile(path, path_length);
	}

	void OnlineRecording::Serialize(char const * destFilePath, size_t pathLen) const
	{
		return pimpl->Serialize(*this, destFilePath, pathLen);
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
