#ifndef XRSLAM_SLIDING_WINDOW_TRACKER_H
#define XRSLAM_SLIDING_WINDOW_TRACKER_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

	class Config;
	class Frame;
	class Map;

	// �ڻ�����ά��frame��subframe�ṹ
	class SlidingWindowTracker {
	public:
		SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
			std::shared_ptr<Config> config);
		~SlidingWindowTracker();

		// ��feature_track_map��frame_id��frame������sliding_window_map��
		// ͬʱ����sliding_window_map�е�IMU��Ϣ���Ӿ���Ϣ
		void mirror_frame(Map *feature_tracking_map, size_t frame_id);
		
		// ���ݴ���֡������֡�Ĺ�ϵ��������֡״̬��
		void localize_newframe();

		// ���ݵ�ͼ�е�����֡���ǻ�������landmark
		void track_landmark();

		// ����map�����еĹؼ�֡��·����Ϣ�����Ż�
		void refine_window();

		// �����йؼ�֡����������ֵʱȥ������֡
		void slide_window();

		// ����slide_window�еĹؼ�֡
		//��1������֡������֡��λ��ϵ
		//��2������֡���ٵ�����Ч��ͼ������
		// ����ֵΪ������map�ؼ�֡�Ƿ����ı�
		bool manage_keyframe();

		// ά��map������֡����֡��ʹ��֡�������ȶ�
		// �����Ż�����֡������֡��λ�ù�ϵ״̬��
		void refine_subwindow();

		// �����Ż���ʵ�ֺ���
		bool track();

		// ��ȡ��ͼ������֡��״̬����ʱ�����λ�ˡ��ٶȡ�ƫ������
		std::tuple<double, PoseState, MotionState> get_latest_state() const;

		// ������ά���Ľ����ڻ����ĵ�ͼ
		std::unique_ptr<Map>	map;

	private:
		std::shared_ptr<Config> config;	// slamϵͳ������Ϣ
	};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
