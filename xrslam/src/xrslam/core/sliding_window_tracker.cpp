#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>

namespace xrslam {

	SlidingWindowTracker::SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
		std::shared_ptr<Config> config)
		: map(std::move(keyframe_map)), config(config) {
		for (size_t j = 1; j < map->frame_num(); ++j) {
			Frame *frame_i = map->get_frame(j - 1);
			Frame *frame_j = map->get_frame(j);
			frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg,
				frame_i->motion.ba, true, true);
		}
	}

	SlidingWindowTracker::~SlidingWindowTracker() = default;

	void SlidingWindowTracker::mirror_frame(Map *feature_tracking_map, size_t frame_id) {
		// 取滑窗中管理的map的最后一帧关键帧
		Frame *keyframe = map->get_frame(map->frame_num() - 1);

		// 取最后一帧关键帧的子帧序列最后一帧
		Frame *new_frame_i = keyframe;
		if (!keyframe->subframes.empty()) {
			new_frame_i = keyframe->subframes.back().get();
		}

		size_t frame_index_i = feature_tracking_map->frame_index_by_id(new_frame_i->id());
		size_t frame_index_j = feature_tracking_map->frame_index_by_id(frame_id);

		if (frame_index_i == nil() || frame_index_j == nil()) {
			return;
		}

		// 获取feature_track_map中index_i与index_j位置的两帧图像帧
		Frame *old_frame_i = feature_tracking_map->get_frame(frame_index_i);
		Frame *old_frame_j = feature_tracking_map->get_frame(frame_index_j);

		// 将frame_id位置与滑窗中最后一帧图像之间的IMU值拷贝给frame_id位置
		std::unique_ptr<Frame> curr_frame = std::move(old_frame_j->clone());
		std::vector<ImuData> &new_data = curr_frame->preintegration.data;
		for (size_t index = frame_index_j - 1; index > frame_index_i; --index) {
			std::vector<ImuData> old_data =
				feature_tracking_map->get_frame(index)->preintegration.data;
			new_data.insert(new_data.begin(), old_data.begin(), old_data.end());
		}

		map->attach_frame(curr_frame->clone());
		Frame *new_frame_j = map->get_frame(map->frame_num() - 1);

		// 更新sliding_window中map的视觉信息
		for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
			if (Track *track = old_frame_i->get_track(ki)) {
				if (size_t kj = track->get_keypoint_index(old_frame_j); kj != nil()) {
					new_frame_i->get_track(ki, map.get())->add_keypoint(new_frame_j, kj);
				}
			}
		}

		new_frame_j->preintegration.integrate(new_frame_j->image->t,
			new_frame_i->motion.bg, new_frame_i->motion.ba, true, true);
		new_frame_j->preintegration.predict(new_frame_i, new_frame_j);
	}

	bool SlidingWindowTracker::track() {
		localize_newframe();

		// map中关键帧状态发生改变
		if (manage_keyframe()) {
			// 三角化恢复一些最新帧的路标点
			track_landmark();

			// 根据当前关键帧信息以及路标点信息优化滑窗
			refine_window();

			// 若滑窗超过设定大小，去掉滑窗中的最老帧
			slide_window();
		}
		// map中关键帧状态不变
		else {
			// 对subframe中的帧进行优化
			refine_subwindow();
		}

		inspect_debug(sliding_window_landmarks, landmarks) {
			std::vector<Landmark> points;
			points.reserve(map->track_num());
			for (size_t i = 0; i < map->track_num(); ++i) {
				if (Track *track = map->get_track(i)) {
					if (track->tag(TT_VALID)) {
						Landmark point;
						point.p = track->get_landmark_point();
						point.triangulated = track->tag(TT_TRIANGULATED);
						points.push_back(point);
					}
				}
			}
			landmarks = std::move(points);
		}

		inspect_debug(sliding_window_current_bg, bg) {
			bg = std::get<2>(get_latest_state()).bg;
		}

		inspect_debug(sliding_window_current_ba, ba) {
			ba = std::get<2>(get_latest_state()).ba;
		}

		return true;
	}

	void SlidingWindowTracker::localize_newframe() {
		auto solver = Solver::create();

		Frame *frame_i = map->get_frame(map->frame_num() - 2);
		if (!frame_i->subframes.empty()) {
			frame_i = frame_i->subframes.back().get();
		}
		Frame *frame_j = map->get_frame(map->frame_num() - 1);

		solver->add_frame_states(frame_j);

		// 添加IMU残差
		solver->put_factor(Solver::create_preintegration_prior_factor(
			frame_i, frame_j, frame_j->preintegration));

		// 添加视觉残差
		for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
			if (Track *track = frame_j->get_track(k)) {
				if (/* track->has_keypoint(frame_i) && */ track->all_tagged(
					TT_VALID, TT_TRIANGULATED)) {
					solver->put_factor(
						Solver::create_reprojection_prior_factor(frame_j, track));
				}
			}
		}

		solver->solve();
	}

	bool SlidingWindowTracker::manage_keyframe() {
		Frame *keyframe_i = map->get_frame(map->frame_num() - 2);	// 次新帧
		Frame *newframe_j = map->get_frame(map->frame_num() - 1);	// 最新帧

		// 1、根据当前帧与上一帧的位置关系，确定滑窗中地图的关键帧
		if (!keyframe_i->subframes.empty()) {
			if (keyframe_i->subframes.back()->tag(FT_NO_TRANSLATION)) {
				if (newframe_j->tag(FT_NO_TRANSLATION)) {
					// [T]...........<-[R]
					//  +-[R]-[R]-[R]
					// ==>
					// [T]
					//  +-[R-R]-[R]-[R]
					/*keyframe_i->subframes.emplace_back(map->detach_frame(map->frame_num() - 1));
					newframe_j->tag(FT_KEYFRAME) = false;*/
				}
				else {
					// [T]...........<-[T]
					//  +-[R]-[R]-[R]
					// ==>
					// [T]........[R]-[T]
					//  +-[R]-[R]
					keyframe_i->subframes.back()->tag(FT_KEYFRAME) = true;
					map->attach_frame(std::move(keyframe_i->subframes.back()), map->frame_num() - 1);
					keyframe_i->subframes.pop_back();
					newframe_j->tag(FT_KEYFRAME) = true;
					return true;
				}
			}
			else {
				if (newframe_j->tag(FT_NO_TRANSLATION)) {
					// [T]...........<-[R]
					//  +-[T]-[T]-[T]
					// ==>
					// [T]........[T]
					//  +-[T]-[T]  +-[R]
					std::unique_ptr<Frame> frame_lifted = std::move(keyframe_i->subframes.back());
					keyframe_i->subframes.pop_back();
					frame_lifted->tag(FT_KEYFRAME) = true;
					newframe_j->tag(FT_KEYFRAME) = false;
					frame_lifted->subframes.emplace_back(map->detach_frame(map->frame_num() - 1));
					map->attach_frame(std::move(frame_lifted));
					return true;
				}
				else {
					// 连续三帧图像都不是纯旋转状态，下一帧图像设为关键帧
					if (keyframe_i->subframes.size() >= 3) {
						// [T]...........<-[T]
						//  +-[T]-[T]-[T]
						// ==>
						// [T]............[T]
						//  +-[T]-[T]-[T]
						newframe_j->tag(FT_KEYFRAME) = true;
						return true;
					}
				}
			}
		}

		// 2、根据最新帧跟踪的有效地图点数量确定最新帧是否为关键帧
		size_t mapped_landmark_count = 0;
		for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
			if (Track *track = newframe_j->get_track(k)) {
				if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
					mapped_landmark_count++;
				}
			}
		}

		bool is_keyframe = mapped_landmark_count <
			config->sliding_window_force_keyframe_landmarks();

#if defined(XRSLAM_IOS)
		is_keyframe = is_keyframe || !newframe_j->tag(FT_NO_TRANSLATION);
#endif

		if (is_keyframe) {
			newframe_j->tag(FT_KEYFRAME) = true;
			return true;
		}
		else {
			newframe_j->tag(FT_KEYFRAME) = false;
			keyframe_i->subframes.emplace_back(
				map->detach_frame(map->frame_num() - 1));
			return false;
		}
	}

	void SlidingWindowTracker::track_landmark() {
		Frame *newframe_j = map->get_frame(map->frame_num() - 1);

		// 三角化地图中最新帧中未三角化的特征点
		for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
			if (Track *track = newframe_j->get_track(k)) {
				if (!track->tag(TT_TRIANGULATED)) {
					if (auto p = track->triangulate()) {
						track->set_landmark_point(p.value());
						track->tag(TT_TRIANGULATED) = true;
						track->tag(TT_VALID) = true;
					}
					else {
						// outlier
						track->landmark.inv_depth = -1.0;
						track->tag(TT_TRIANGULATED) = false;
						track->tag(TT_VALID) = false;
					}
				}
			}
		}
	}

	void SlidingWindowTracker::refine_window() {
		Frame *keyframe_i = map->get_frame(map->frame_num() - 2);	// 次新帧 
		Frame *keyframe_j = map->get_frame(map->frame_num() - 1);	// 最新帧

		auto solver = Solver::create();
		if (!map->marginalization_factor) {
			map->marginalization_factor =
				Solver::create_marginalization_factor(map.get());
		}

		// 1、添加优化量
		// 1.1、添加相机状态量
		for (size_t i = 0; i < map->frame_num(); ++i) {
			Frame *frame = map->get_frame(i);
			solver->add_frame_states(frame);
		}

		// 1.2、添加路标点优化量
		std::unordered_set<Track *> visited_tracks;
		for (size_t i = 0; i < map->frame_num(); ++i) {
			Frame *frame = map->get_frame(i);
			for (size_t j = 0; j < frame->keypoint_num(); ++j) {
				Track *track = frame->get_track(j);
				if (!track)
					continue;
				if (visited_tracks.count(track) > 0)
					continue;
				visited_tracks.insert(track);
				if (!track->tag(TT_VALID))
					continue;
				if (!track->first_frame()->tag(FT_KEYFRAME))
					continue;
				solver->add_track_states(track);
			}
		}

		// 2、添加优化残差
		// 2.1、添加边缘化残差
		solver->add_factor(map->marginalization_factor.get());

		// 2.2、添加视觉残差
		for (size_t i = 0; i < map->frame_num(); ++i) {
			Frame *frame = map->get_frame(i);
			for (size_t j = 0; j < frame->keypoint_num(); ++j) {
				Track *track = frame->get_track(j);
				if (!track)
					continue;
				if (!track->all_tagged(TT_VALID, TT_TRIANGULATED))
					continue;
				if (!track->first_frame()->tag(FT_KEYFRAME))
					continue;
				if (frame == track->first_frame())
					continue;

				solver->add_factor(frame->reprojection_error_factors[j].get());
			}
		}

		// 2.3、添加IMU残差
		for (size_t j = 1; j < map->frame_num(); ++j) {
			Frame *frame_i = map->get_frame(j - 1);
			Frame *frame_j = map->get_frame(j);

			// 获取关键帧i与关键帧j之间的所有IMU数据
			frame_j->keyframe_preintegration = frame_j->preintegration;
			if (!frame_i->subframes.empty()) {
				std::vector<ImuData> imu_data;
				for (size_t k = 0; k < frame_i->subframes.size(); ++k) {
					auto &sub_imu_data = frame_i->subframes[k]->preintegration.data;
					imu_data.insert(imu_data.end(), sub_imu_data.begin(),
						sub_imu_data.end());
				}
				frame_j->keyframe_preintegration.data.insert(
					frame_j->keyframe_preintegration.data.begin(), imu_data.begin(),
					imu_data.end());
			}

			if (frame_j->keyframe_preintegration.integrate(
				frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true, true)) {
				solver->put_factor(Solver::create_preintegration_error_factor(
					frame_i, frame_j, frame_j->keyframe_preintegration));
			}
		}

		// 3、开始进行非线性优化
		solver->solve();

		// 4、根据优化后的姿态和路标点计算当前map中地图点的有效性
		for (size_t k = 0; k < map->track_num(); ++k) {
			Track *track = map->get_track(k);
			if (track->tag(TT_TRIANGULATED)) {
				bool is_valid = true;
				auto x = track->get_landmark_point();
				double rpe = 0.0;
				double rpe_count = 0.0;
				for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
					// 在关键帧中对路标点进行重投影验证
					if (!frame->tag(FT_KEYFRAME)) {
						continue;
					}
					
					// 获取路标点在当前关键帧下的相机坐标系坐标
					PoseState pose = frame->get_pose(frame->camera);
					vector<3> y = pose.q.conjugate() * (x - pose.p);
					
					// 若路标点出现在相机的后面或无穷远处表明当前点无效
					if (y.z() <= 1.0e-3 || y.z() > 50) {
						is_valid = false;
						break;
					}

					// 获取当前路标点在当前关键帧中的重投影误差
					rpe += (apply_k(y, frame->K) -
						apply_k(frame->get_keypoint(keypoint_index), frame->K))
						.norm();
					rpe_count += 1.0;
				}

				// 重投影误差且无深度异常视当前路标点为有效点
				is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) < 3.0);
				track->tag(TT_VALID) = is_valid;
			}
			else {
				track->landmark.inv_depth = -1.0;
			}
		}

		// 5、去除地图中无效的路标点
		map->prune_tracks([](const Track *track) { return !track->tag(TT_VALID); });
	}

	void SlidingWindowTracker::slide_window() {
		while (map->frame_num() > config->sliding_window_size()) {
			Frame *frame = map->get_frame(0);
			
			// 去除sliding_window地图中需去除的关键帧子帧的所有路标点信息
			for (size_t i = 0; i < frame->subframes.size(); ++i) {
				map->untrack_frame(frame->subframes[i].get());
			}
			
			// 去除sliding_window地图中需去除的关键帧信息
			map->marginalize_frame(0);
		}
	}

	void SlidingWindowTracker::refine_subwindow() {
		// 1、获取当前地图中的最新帧
		Frame *frame = map->get_frame(map->frame_num() - 1);
		if (frame->subframes.empty()) {
			return;
		}

		if (frame->subframes[0]->tag(FT_NO_TRANSLATION)) {
			// 若subframes太多，适当精减一些，并把imu数据拷贝到对应帧中
			if (frame->subframes.size() >= 9) {
				for (size_t i = frame->subframes.size() / 3; i > 0; --i) {
					Frame *tgt_frame = frame->subframes[i * 3 - 1].get();
					std::vector<ImuData> imu_data;
					for (size_t j = i * 3 - 1; j > (i - 1) * 3; --j) {
						Frame *src_frame = frame->subframes[j - 1].get();
						imu_data.insert(imu_data.begin(),
							src_frame->preintegration.data.begin(),
							src_frame->preintegration.data.end());
						map->untrack_frame(src_frame);
						frame->subframes.erase(frame->subframes.begin() + (j - 1));
					}
					tgt_frame->preintegration.data.insert(
						tgt_frame->preintegration.data.begin(), imu_data.begin(),
						imu_data.end());
				}
			}

			// 调整subframes后，对subwindow进行优化
			auto solver = Solver::create();

			// 由于是对subwindow优化，因此需要对frame进行固定
			// 从而保证frame在滑窗地图中状态量的连续性
			frame->tag(FT_FIX_POSE) = true;
			frame->tag(FT_FIX_MOTION) = true;

			// 添加IMU残差
			solver->add_frame_states(frame);
			for (size_t i = 0; i < frame->subframes.size(); ++i) {
				Frame *subframe = frame->subframes[i].get();
				solver->add_frame_states(subframe);
				Frame *prev_frame =
					(i == 0 ? frame : frame->subframes[i - 1].get());
				subframe->preintegration.integrate(
					subframe->image->t, prev_frame->motion.bg,
					prev_frame->motion.ba, true, true);
				solver->put_factor(Solver::create_preintegration_error_factor(
					prev_frame, subframe, subframe->preintegration));
			}

			// 添加视觉残差
			Frame *last_subframe = frame->subframes.back().get();
			for (size_t k = 0; k < last_subframe->keypoint_num(); ++k) {
				if (Track *track = last_subframe->get_track(k)) {
					if (track->tag(TT_VALID)) {
						if (track->tag(TT_TRIANGULATED)) {
							solver->put_factor(
								Solver::create_reprojection_prior_factor(
									last_subframe, track));
						}
						else {
							solver->put_factor(
								Solver::create_rotation_prior_factor(
									last_subframe, track));
						}
					}
				}
			}

			solver->solve();
			frame->tag(FT_FIX_POSE) = false;
			frame->tag(FT_FIX_MOTION) = false;
		}
		// 若frame对应的subframe并不多，此时直优化subwindow的状态量
		else {
			auto solver = Solver::create();
			frame->tag(FT_FIX_POSE) = true;
			frame->tag(FT_FIX_MOTION) = true;

			// 添加IMU残差
			solver->add_frame_states(frame);
			for (size_t i = 0; i < frame->subframes.size(); ++i) {
				Frame *subframe = frame->subframes[i].get();
				solver->add_frame_states(subframe);
				Frame *prev_frame =
					(i == 0 ? frame : frame->subframes[i - 1].get());
				subframe->preintegration.integrate(
					subframe->image->t, prev_frame->motion.bg,
					prev_frame->motion.ba, true, true);

				solver->put_factor(Solver::create_preintegration_error_factor(
					prev_frame, subframe, subframe->preintegration));

				// 添加视觉残差
				for (size_t k = 0; k < subframe->keypoint_num(); ++k) {
					if (Track *track = subframe->get_track(k)) {
						if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
							if (track->first_frame()->tag(FT_KEYFRAME)) {
								solver->put_factor(
									Solver::create_reprojection_prior_factor(
										subframe, track));
							}
							else if (track->first_frame()->id() > frame->id()) {
								solver->add_factor(
									frame->reprojection_error_factors[k].get());
							}
						}
					}
				}
			}

			solver->solve();
			frame->tag(FT_FIX_POSE) = false;
			frame->tag(FT_FIX_MOTION) = false;
		}
	}

	std::tuple<double, PoseState, MotionState>
		SlidingWindowTracker::get_latest_state() const {
		const Frame *frame = map->get_frame(map->frame_num() - 1);
		if (!frame->subframes.empty()) {
			frame = frame->subframes.back().get();
		}
		return { frame->image->t, frame->pose, frame->motion };
	}

} // namespace xrslam
