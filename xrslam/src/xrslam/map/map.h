#ifndef XRSLAM_MAP_H
#define XRSLAM_MAP_H

#include <xrslam/common.h>

namespace xrslam {

	class Frame;
	class Track;
	class MarginalizationFactor;
	class VirtualObjectManager;
	class Localizer;

	class Map {
		friend class Track;
		struct construct_by_map_t;

	public:
		Map();
		virtual ~Map();

		void clear();

		size_t frame_num() const { return frames.size(); }

		Frame* get_frame(size_t index) const { return frames[index].get(); }

		void attach_frame(std::unique_ptr<Frame> frame, size_t position = nil());
		std::unique_ptr<Frame> detach_frame(size_t index);

		// 将frame中所有的特征点从track中删除
		void untrack_frame(Frame *frame);
		// 将frame从当前map中删除
		void erase_frame(size_t index);
		void marginalize_frame(size_t index);

		size_t frame_index_by_id(size_t id) const;

		size_t track_num() const { return tracks.size(); }

		Track *get_track(size_t index) const { return tracks[index].get(); }

		Track *create_track();
		void erase_track(Track *track);

		// 修剪掉不符合condition情况的track
		void prune_tracks(const std::function<bool(const Track *)> &condition);

		Track* get_track_by_id(size_t id) const;

		// XR相关
		void create_virtual_object_manager(Localizer *localizer);
		void create_virtual_object_manager();
		bool is_virtual_object_manager_running() {
			return virtual_object_manager.get() != nullptr;
		}
		size_t create_virtual_object();
		void update_virtual_objects();
		OutputObject get_virtual_object_pose_by_id(size_t id) const;
		OutputObject get_virtual_object_pose(size_t index) const;
		size_t virtual_object_num() const;

		std::unique_lock<std::mutex> lock() const {
			return std::unique_lock(map_mutex);
		}

		std::unique_ptr<MarginalizationFactor> marginalization_factor;

	private:
		void recycle_track(Track *track);

		std::deque<std::unique_ptr<Frame>>		frames;						// map中存储的所有帧,从队列的尾部插入元素
		std::vector<std::unique_ptr<Track>>		tracks;						// map中存储的所有特征点跟踪器
		std::map<size_t, Track *>				track_id_map;				// TODO
		std::unique_ptr<VirtualObjectManager>	virtual_object_manager;		// map中存储的所有虚拟对象
		mutable std::mutex						map_mutex;
	};

} // namespace xrslam

#endif // XRSLAM_MAP_H
