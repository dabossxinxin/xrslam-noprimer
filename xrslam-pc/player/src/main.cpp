#include <argparse.hpp>
#include <chrono>
#include <dataset_reader.h>
#include <functional>
#include <iostream>
#include <opencv_painter.h>
#include <trajectory_writer.h>
#include <xrslam/extra/opencv_image.h>
#include <xrslam/extra/yaml_config.h>
#include <xrslam/inspection.h>
#include <xrslam/xrslam.h>

#ifndef XRSLAM_PC_HEADLESS_ONLY

#include <glbinding-aux/types_to_string.h>
#include <lightvis.h>
#include <nuklear.h>

class PlayerVisualizer : public lightvis::LightVis {
private:
	std::unique_ptr<xrslam::XRSLAM>					xrslam;		// SLAM算法指针
	std::unique_ptr<DatasetReader>					input;		// 数据输入指针
	std::vector<std::unique_ptr<TrajectoryWriter>>	outputs;	// 数据输出指针

	int is_playing			= 0;
	bool has_gyroscope		= false;
	bool has_accelerometer	= false;

	std::vector<Eigen::Vector3f>	trajectory;			// slam轨迹
	std::vector<Eigen::Vector3f>	landmarks;			// slam路标点
	std::vector<Eigen::Vector3f>	global_landmarks;	// 全局路标点
	Eigen::Vector4f					trajectory_color;	// 轨迹颜色
	Eigen::Vector4f					camera_color;		// 相机颜色
	std::vector<Eigen::Vector4f>	landmarks_color;	// 路标点颜色
	std::vector<Eigen::Vector4f>	global_landmark_color;
	std::deque<Eigen::Vector3d>		bg_list;			// 陀螺仪偏置
	std::deque<Eigen::Vector3d>		ba_list;			// 加速度计偏置

	cv::Mat feature_tracker_cvimage;
	std::unique_ptr<xrslam::InspectPainter> feature_tracker_painter;
	std::unique_ptr<lightvis::Image>		feature_tracker_image;

	std::vector<Eigen::Vector3f>	positions;			// 相机空间位置序列
	xrslam::quaternion				latest_camera_q;	// 最近计算得到的相机姿态
	xrslam::vector<3>				latest_camera_p;	// 最近计算得到的相机位置
	std::shared_ptr<xrslam::extra::YamlConfig> config;	// slam系统配置文件指针

	std::vector<double> bg_x, bg_y, bg_z, ba_x, ba_y, ba_z;

public:
	PlayerVisualizer(std::unique_ptr<xrslam::XRSLAM> xrslam,
		std::unique_ptr<DatasetReader> input,
		std::vector<std::unique_ptr<TrajectoryWriter>> outputs,
		bool play,
		std::shared_ptr<xrslam::extra::YamlConfig> yaml_config)
		: LightVis("XRSLAM Player", 1200, 720), xrslam(std::move(xrslam)),
		input(std::move(input)), outputs(std::move(outputs)),
		config(yaml_config) {

		trajectory_color = { 0.0, 1.0, 1.0, 1.0 };	// 红、绿、蓝、不透明度
		camera_color = { 1.0, 0.0, 0.0, 1.0 };		// 红、绿、蓝、不透明度

		feature_tracker_painter =
			std::make_unique<OpenCvPainter>(feature_tracker_cvimage);
		inspect_debug(feature_tracker_painter, painter) {
			painter = feature_tracker_painter.get();
		}
		is_playing = (int)play;
		bg_x.resize(250, 0.5);
		bg_y.resize(250, 0.5);
		bg_z.resize(250, 0.5);
		ba_x.resize(250, 0.5);
		ba_y.resize(250, 0.5);
		ba_z.resize(250, 0.5);
	}

	void load() override {
		feature_tracker_image = std::make_unique<lightvis::Image>();

		add_trajectory(trajectory, trajectory_color);
		add_points(landmarks, landmarks_color);
		add_points(global_landmarks, global_landmark_color);

		add_image(feature_tracker_image.get());	// 添加特征提取视图
		add_separator();
		add_graph(bg_x);						// 添加陀螺仪偏置视图
		add_separator();
		add_graph(bg_y);
		add_separator();
		add_graph(bg_z);
		add_separator();
		add_graph(ba_x);						// 添加加速度计偏置视图
		add_separator();
		add_graph(ba_y);
		add_separator();
		add_graph(ba_z);
	}

	void unload() override { feature_tracker_image.reset(); }

	bool step() {
		while (true) {
			DatasetReader::NextDataType next_type;
			while ((next_type = input->next()) == DatasetReader::AGAIN) {
				continue;
			}
			switch (next_type) {
			case DatasetReader::GYROSCOPE: {
				has_gyroscope = true;
				auto[t, w] = input->read_gyroscope();
				xrslam->track_gyroscope(t, w.x(), w.y(), w.z());

			} break;
			case DatasetReader::ACCELEROMETER: {
				has_accelerometer = true;
				auto[t, a] = input->read_accelerometer();
				xrslam->track_accelerometer(t, a.x(), a.y(), a.z());

			} break;
			case DatasetReader::CAMERA: {
				auto image = input->read_image();
				if (has_accelerometer && has_gyroscope) {
					auto pose = xrslam->track_camera(image);
					if (!pose.q.coeffs().isZero()) {
						Eigen::Vector3f p =
							(pose.p - pose.q * config->camera_to_body_translation()).cast<float>();
						trajectory.push_back(p);
						location() = { p.x(), p.y(), p.z() };
						for (auto &output : outputs) {
							output->write_pose(image->t, pose);
						}
						
						latest_camera_q = pose.q * config->camera_to_body_rotation().conjugate();
						latest_camera_p = pose.p - pose.q * config->camera_to_body_translation();
						add_camera(positions, latest_camera_p, latest_camera_q,
							config->camera_intrinsic(), camera_color);	// 可视化当前相机姿态

						inspect_debug(sliding_window_landmarks, swlandmarks) {
							auto points =
								std::any_cast<std::vector<xrslam::Landmark>>(
									swlandmarks);
							landmarks.clear();
							landmarks_color.clear();
							for (auto &p : points) {
								landmarks.push_back(p.p.cast<float>());
								if (p.triangulated) {
									landmarks_color.emplace_back(0.0, 1.0, 0.0,
										0.5);	// 三角化后的点设置为绿色
								}
								else {
									landmarks_color.emplace_back(1.0, 0.0, 0.0,
										0.5);	// 未三角化的点设置为红色
								}
							}
						}

						inspect_debug(global_map_landmarks, gmlandmarks) {
							auto points = std::any_cast<std::vector<xrslam::Landmark>>(gmlandmarks);
							global_landmarks.clear();
							for (auto& p : points) {
								if (p.triangulated) {
									global_landmarks.emplace_back(p.p.cast<float>());
									global_landmark_color.emplace_back(1.0, 1.0, 1.0, 0.5);
								}
							}
						}

						inspect_debug(sliding_window_current_bg, bg) {
							if (bg.has_value()) {
								bg_list.emplace_back(
									std::any_cast<xrslam::vector<3>>(bg));
								while (bg_list.size() > 250) {
									bg_list.pop_front();
								}
							}
						}
						inspect_debug(sliding_window_current_ba, ba) {
							if (ba.has_value()) {
								ba_list.emplace_back(
									std::any_cast<xrslam::vector<3>>(ba));
								while (ba_list.size() > 250) {
									ba_list.pop_front();
								}
							}
						}
					}
				}
				return true;
			} break;
			case DatasetReader::END: {
				return false;
			} break;
			default: {
			} break;
			}
		}
	}

	void gui(void *ctx, int w, int h) override {
		feature_tracker_image->update_image(feature_tracker_cvimage);
		{
			auto xt = bg_x.rbegin(), yt = bg_y.rbegin(), zt = bg_z.rbegin();
			for (auto it = bg_list.rbegin(); it != bg_list.rend();
				it++, xt++, yt++, zt++) {
				(*xt) = it->x() / 0.05 + 0.5;
				(*yt) = it->y() / 0.05 + 0.5;
				(*zt) = it->z() / 0.05 + 0.5;
			}
		}
		{
			auto xt = ba_x.rbegin(), yt = ba_y.rbegin(), zt = ba_z.rbegin();
			for (auto it = ba_list.rbegin(); it != ba_list.rend();
				it++, xt++, yt++, zt++) {
				(*xt) = it->x() / 0.1 + 0.5;
				(*yt) = it->y() / 0.1 + 0.5;
				(*zt) = it->z() / 0.1 + 0.5;
			}
		}
		LightVis::gui(ctx, w, h);

		auto *context = (nk_context *)(ctx);

		if (nk_begin(context, "Controls", nk_rect(0, h - 40, 240, 40),
			NK_WINDOW_NO_SCROLLBAR)) {
			nk_layout_row_static(context, 40, 80, 3);
			if (is_playing) {
				is_playing = step();
			}
			if (is_playing) {
				if (nk_button_label(context, "Playing")) {
					is_playing = false;
				}
			}
			else {
				if (nk_button_label(context, "Stopped")) {
					is_playing = true;
				}
				nk_button_push_behavior(context, NK_BUTTON_REPEATER);
				if (nk_button_label(context, "Forward")) {
					step();
				}
				nk_button_pop_behavior(context);
				nk_button_push_behavior(context, NK_BUTTON_DEFAULT);
				if (nk_button_label(context, "Step")) {
					step();
				}
				nk_button_pop_behavior(context);
			}
		}
		nk_end(context);
	}

	void draw(int w, int h) override {
		auto err = gl::glGetError();
		if (err != gl::GL_NONE) {
			std::cout << err << std::endl;
			exit(0);
		}
	}
};

#endif

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("XRSLAM Player");
    program.add_argument("-c", "--config")
        .help("Configuration YAML file.")
        .nargs(1);
    program.add_argument("--csv").help("Save CSV-format trajectory.").nargs(1);
    program.add_argument("--tum").help("Save TUM-format trajectory.").nargs(1);
    program.add_argument("-p", "--play")
        .help("Start playing immediately.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-H", "--headless")
        .help("Don't show visualization, implies -P.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("input").help("input file");
    program.parse_args(argc, argv);

    std::string input = program.get<std::string>("input");
    std::string config = program.get<std::string>("-c");
    std::string csv_output = program.get<std::string>("--csv");
    std::string tum_output = program.get<std::string>("--tum");
    bool play = program.get<bool>("-p");
    bool headless = program.get<bool>("-H");

    std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
        std::make_shared<xrslam::extra::YamlConfig>(config);
    std::unique_ptr<xrslam::XRSLAM> xrslam =
        std::make_unique<xrslam::XRSLAM>(yaml_config);

    std::unique_ptr<DatasetReader> reader = DatasetReader::create_reader(input);
    if (!reader) {
        fprintf(stderr, "Cannot open \"%s\"\n", input.c_str());
        return EXIT_FAILURE;
    }

    std::vector<std::unique_ptr<TrajectoryWriter>> outputs;
    if (csv_output.length() > 0) {
        outputs.emplace_back(std::make_unique<CsvTrajectoryWriter>(csv_output));
    }
    if (tum_output.length() > 0) {
        outputs.emplace_back(std::make_unique<TumTrajectoryWriter>(tum_output));
    }

    if (headless) {
        bool has_gyroscope = false, has_accelerometer = false;
        outputs.emplace_back(std::make_unique<ConsoleTrajectoryWriter>());
        DatasetReader::NextDataType next_type;
        while ((next_type = reader->next()) != DatasetReader::END) {
            switch (next_type) {
            case DatasetReader::AGAIN:
                continue;
            case DatasetReader::GYROSCOPE: {
                has_gyroscope = true;
                auto [t, w] = reader->read_gyroscope();
                xrslam->track_gyroscope(t, w.x(), w.y(), w.z());
            } break;
            case DatasetReader::ACCELEROMETER: {
                has_accelerometer = true;
                auto [t, a] = reader->read_accelerometer();
                xrslam->track_accelerometer(t, a.x(), a.y(), a.z());
            } break;
            case DatasetReader::CAMERA: {
                auto image = reader->read_image();
                if (has_accelerometer && has_gyroscope) {
                    auto pose = xrslam->track_camera(image);
                    if (!pose.q.coeffs().isZero()) {
                        for (auto &output : outputs) {
                            output->write_pose(image->t, pose);
                        }
                    }
                }
            } break;
            default: {
            } break;
            }
        }
    } else {
#ifndef XRSLAM_PC_HEADLESS_ONLY
        PlayerVisualizer vis(std::move(xrslam), std::move(reader),
                             std::move(outputs), play, yaml_config);
        vis.show();
		return lightvis::main();
#else
        fprintf(stderr, "Current build can only run in headless mode.");
        return EXIT_FAILURE;
#endif
    }

    return EXIT_SUCCESS;
}
