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

		// 主线程将frame数据传送到feature_track线程中
		void track_frame(std::unique_ptr<Frame> frame);

		void mirror_map(Map *sliding_window_tracker_map);
		void synchronize_keymap(Map *sliding_window_tracker_map);
		void mirror_keyframe(Map *sliding_window_tracker_map, size_t frame_id);
		void mirror_lastframe(Map *sliding_window_tracker_map);
		void attach_latest_frame(Frame *frame);
		void manage_keymap();
		void solve_pnp();

		// 估计量是否有效以及估计量的值都保存在optional中
		std::optional<std::tuple<double, PoseState, MotionState>>
			get_latest_state() const;

		std::unique_ptr<Map> map;		// slam系统中维护的地图
		std::unique_ptr<Map> keymap;	// slam系统中维护的关键帧地图

	private:
		XRSLAM::Detail*						detail;		// slam系统handle
		std::shared_ptr<Config>				config;		// slam系统配置
		std::deque<std::unique_ptr<Frame>>	frames;		// 待处理图像帧

		// mutable突破const的限制，在const函数中也可改变
		mutable std::mutex					latest_pose_mutex;

		std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
	};

} // namespace xrslam

#endif // XRSLAM_FEATURE_TRACKER_H
