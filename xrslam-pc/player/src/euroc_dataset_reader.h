#ifndef XRSLAM_PC_EUROC_DATASET_READER_H
#define XRSLAM_PC_EUROC_DATASET_READER_H

#include <dataset_reader.h>
#include <deque>
#include <string>
#include <fstream>

// euroc格式数据读取器
class EurocDatasetReader : public DatasetReader {
  public:
    EurocDatasetReader(const std::string &filename);
    NextDataType next() override;

	// 从成员变量中读取图像数据并去畸变
    std::shared_ptr<xrslam::Image> read_image() override;

	// 从成员变量中读取陀螺仪数据
    std::pair<double, xrslam::vector<3>> read_gyroscope() override;

	// 从成员变量中读取加速度计数据
    std::pair<double, xrslam::vector<3>> read_accelerometer() override;

  private:
    std::deque<std::pair<double, NextDataType>>			all_data;
    std::deque<std::pair<double, xrslam::vector<3>>>	gyroscope_data;
    std::deque<std::pair<double, xrslam::vector<3>>>	accelerometer_data;
    std::deque<std::pair<double, std::string>>			image_data;
};

// 读取相机数据
struct CameraCsv {
    struct CameraData {
        double t;
        std::string filename;
    };

    std::vector<CameraData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
			std::fscanf(csv, "%2047[^\r]\n", header_line);
            char filename_buffer[2048];
            CameraData item;
            while (!feof(csv)) {
                std::memset(filename_buffer, 0, 2048);
                if (std::fscanf(csv, "%lf,%2047[^\r]\r\n", &item.t,
                           filename_buffer) != 2) {
                    break;
                }
                item.t *= 1e-9;
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

	void load_new(const std::string& filename) {
		items.clear();
		std::ifstream in;
		in.open(filename.c_str());

		std::string header_line;
		std::getline(in, header_line);

		while (!in.eof()) {
			std::string sdata;
			std::getline(in, sdata);
			if (!sdata.empty()) {
				CameraData item;
				std::stringstream stream;
				stream << sdata;

				stream >> item.t;
				stream >> item.filename;
				item.t *= 1e-9;
				items.emplace_back(std::move(item));
			}
		}
		in.close();
	}

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            std::fputs("#t[ns],filename[string]\n", csv);
            for (auto item : items) {
				std::fprintf(csv, "%lld,%s\n", int64_t(item.t * 1e9),
					item.filename.c_str());
            }
            fclose(csv);
        }
    }
};

// 读取IMU数据
struct ImuCsv {
    struct ImuData {
        double t;
        struct {
            double x;
            double y;
            double z;
        } w;
        struct {
            double x;
            double y;
            double z;
        } a;
    };

    std::vector<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            fscanf(csv, "%2047[^\r]\r\n", header_line);
            ImuData item;
            while (!feof(csv) &&
                   fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n", &item.t,
                          &item.w.x, &item.w.y, &item.w.z, &item.a.x, &item.a.y,
                          &item.a.z) == 7) {
                item.t *= 1e-9;
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

	void load_new(const std::string& filename) {
		items.clear();
		std::ifstream in;
		in.open(filename.c_str());

		std::string header_line;
		std::getline(in, header_line);

		while (!in.eof()) {
			std::string sdata;
			std::getline(in, sdata);
			if (!sdata.empty()) {
				ImuData item;
				std::stringstream stream;
				stream << sdata;

				double t, wx, wy, wz, ax, ay, az;
				stream >> t >> wx >> wy >> wz
					>> ax >> ay >> az;

				item.t = (t)* 1e-9;
				item.w.x = (wx);
				item.w.y = (wy);
				item.w.z = (wz);
				item.a.x = (ax);
				item.a.y = (ay);
				item.a.z = (az);

				items.emplace_back(std::move(item));
			}
		}
		in.close();
	}

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[ns],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/"
                  "s:double],a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/"
                  "s^2:double]\n",
                  csv);
            for (auto item : items) {
                fprintf(csv, "%lld,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n",
                        int64_t(item.t * 1e9), item.w.x, item.w.y, item.w.z,
                        item.a.x, item.a.y, item.a.z);
            }
            fclose(csv);
        }
    }
};

#endif
