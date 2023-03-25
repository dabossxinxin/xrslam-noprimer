#ifndef XRSLAM_FEATURE_TRACKER_H
#define XRSLAM_FEATURE_TRACKER_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/worker.h>

namespace xrslam {

	class FeatureTracker : public Worker {
	public:
		FeatureTracker(XRSLAM::Detail *detail, std::shared_ptr<Config> config);
		~FeatureTracker();

		bool empty() const override { return frames.empty(); }

		void work(std::unique_lock<std::mutex> &l) override;

		// ���߳̽�frame���ݴ��͵�feature_track�߳���
		void track_frame(std::unique_ptr<Frame> frame);

		void mirror_map(Map *sliding_window_tracker_map);
		void synchronize_keymap(Map *sliding_window_tracker_map);
		void mirror_keyframe(Map *sliding_window_tracker_map, size_t frame_id);
		void mirror_lastframe(Map *sliding_window_tracker_map);
		void attach_latest_frame(Frame *frame);
		void manage_keymap();
		void solve_pnp();

		// �������Ƿ���Ч�Լ���������ֵ��������optional��
		std::optional<std::tuple<double, PoseState, MotionState>>
			get_latest_state() const;

		std::unique_ptr<Map> map;		// slamϵͳ��ά���ĵ�ͼ
		std::unique_ptr<Map> keymap;	// slamϵͳ��ά���Ĺؼ�֡��ͼ

	private:
		XRSLAM::Detail*						detail;		// slamϵͳhandle
		std::shared_ptr<Config>				config;		// slamϵͳ����
		std::deque<std::unique_ptr<Frame>>	frames;		// ������ͼ��֡

		// mutableͻ��const�����ƣ���const������Ҳ�ɸı�
		mutable std::mutex					latest_pose_mutex;

		std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
	};

} // namespace xrslam

#endif // XRSLAM_FEATURE_TRACKER_H
