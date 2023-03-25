#include <xrslam/extra/opencv_image.h>
#include <xrslam/extra/poisson_disk_filter.h>
#include <xrslam/extra/yaml_config.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>

#include <gtest/gtest.h>

using namespace xrslam;

std::shared_ptr<xrslam::Image> fun_read_image(std::string filename) {
	cv::Mat img_distorted = cv::imread(filename, cv::IMREAD_GRAYSCALE);
	cv::Mat img;
	cv::Mat dist_coeffs = (cv::Mat_<float>(4, 1) << -0.28340811, 0.07395907,
		0.00019359, 1.76187114e-05);
	cv::Mat K = (cv::Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296,
		248.375, 0, 0, 1);
	cv::undistort(img_distorted, img, K, dist_coeffs);
	std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
		std::make_shared<xrslam::extra::OpenCvImage>();
	opencv_image->image = img;
	return opencv_image;
}

void fun_feature_track()
{
	std::string config = "E:\\Xiongxinxin\\xrslam\\configs\\euroc.yaml";
	std::string filename1 = "E:\\Xiongxinxin\\xrslam\\xrslam-test\\data\\1403715282262142976.png";
	std::string filename2 = "E:\\Xiongxinxin\\xrslam\\xrslam-test\\data\\1403715282312143104.png";

	std::unique_ptr<Frame> frame = std::make_unique<Frame>();
	std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
		std::make_shared<xrslam::extra::YamlConfig>(config);
	frame->K = yaml_config->camera_intrinsic();
	frame->image = fun_read_image(filename1);
	frame->image->preprocess();
	frame->detect_keypoints(yaml_config.get());

	std::cout << "keypoint num:" << frame->keypoint_num() << std::endl;

	std::unique_ptr<Map> map = std::make_unique<Map>();
	map->attach_frame(std::move(frame));

	Frame* last_frame = map->get_frame(0);
	std::unique_ptr<Frame> curr_frame = std::make_unique<Frame>();
	curr_frame->K = yaml_config->camera_intrinsic();
	curr_frame->image = fun_read_image(filename2);
	curr_frame->image->preprocess();
	last_frame->track_keypoints(curr_frame.get(), yaml_config.get());

	int count = 0;
	for (int it = 0; it < curr_frame->keypoint_num(); ++it) {
		if (Track *track = curr_frame->get_track(it)) {
			count++;
		}
	}

	std::cout << "track count:" << count << std::endl;
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

	/*fun_feature_track();*/
}
