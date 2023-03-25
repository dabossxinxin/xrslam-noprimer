#ifndef XRSLAM_SLIDING_WINDOW_TRACKER_H
#define XRSLAM_SLIDING_WINDOW_TRACKER_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

	class Config;
	class Frame;
	class Map;

	// 在滑窗中维护frame的subframe结构
	class SlidingWindowTracker {
	public:
		SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
			std::shared_ptr<Config> config);
		~SlidingWindowTracker();

		// 将feature_track_map中frame_id的frame拷贝到sliding_window_map中
		// 同时更新sliding_window_map中的IMU信息与视觉信息
		void mirror_frame(Map *feature_tracking_map, size_t frame_id);
		
		// 根据次新帧与最新帧的关系计算最新帧状态量
		void localize_newframe();

		// 根据地图中的最新帧三角化出更多landmark
		void track_landmark();

		// 根据map中所有的关键帧与路标信息进行优化
		void refine_window();

		// 滑窗中关键帧数量超过阈值时去掉最老帧
		void slide_window();

		// 管理slide_window中的关键帧
		//【1】次新帧与最新帧的位关系
		//【2】最新帧跟踪到的有效地图点数量
		// 返回值为滑窗中map关键帧是否发生改变
		bool manage_keyframe();

		// 维护map中最新帧的子帧，使子帧数据量稳定
		// 并且优化最新帧与其子帧的位置关系状态量
		void refine_subwindow();

		// 滑窗优化的实现函数
		bool track();

		// 获取地图中最新帧的状态量【时间戳、位姿、速度、偏置量】
		std::tuple<double, PoseState, MotionState> get_latest_state() const;

		// 滑窗中维护的仅用于滑窗的地图
		std::unique_ptr<Map>	map;

	private:
		std::shared_ptr<Config> config;	// slam系统配置信息
	};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
