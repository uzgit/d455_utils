#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <librealsense2/rs.hpp>
#include <unistd.h>
#include "./include/rotation_estimator.h"
//#include <Eigen/Quaternion.h>
//#include <boost/math/quaternion.hpp>

#define DEPTH_WIDTH  640
#define DEPTH_HEIGHT 480
#define DEPTH_FRAMERATE 15
#define COLOR_WIDTH  640
#define COLOR_HEIGHT 480
#define COLOR_FRAMERATE 15

#define TRANSFORM 1
#define VISUALIZE 1

int main()
{
	// say hello
	std::cout << "Starting D455..." << std::endl;

	rotation_estimator rotation_estimate;

	// set up the streams
	rs2::config config;
	config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FRAMERATE);
	config.enable_stream(RS2_STREAM_COLOR, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_BGR8, COLOR_FRAMERATE);
	config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	config.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);

	// create a pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);

	// get the depth sensor and its scaling factor
	rs2::device device = pipeline_profile.get_device();
	rs2::depth_sensor depth_sensor = device.query_sensors().front().as<rs2::depth_sensor>();
	float depth_scale = depth_sensor.get_depth_scale();

#if VISUALIZE
	// set up the windows
	cv::String depth_window_name = "Depth Stream";
	cv::String color_window_name = "Color Stream";
	cv::namedWindow(color_window_name);
	cv::namedWindow(depth_window_name);
#endif

	std::cout << std::setprecision(2);

	while(true)
	{
		// wait for some frames to arrive from the camera
		rs2::frameset frames = pipeline.wait_for_frames();

		for( rs2::frame frame : frames )
		{
			if( frame.is<rs2::motion_frame>() )
			{
				rs2::motion_frame motion_frame = frame.as<rs2::motion_frame>();

				if (motion_frame && motion_frame.get_profile().stream_type() == RS2_STREAM_GYRO && motion_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
				{
					double timestamp = motion_frame.get_timestamp();
					rs2_vector gyro_data = motion_frame.get_motion_data();
					rotation_estimate.process_gyro(gyro_data, timestamp);
				}
				else if (motion_frame && motion_frame.get_profile().stream_type() == RS2_STREAM_ACCEL && motion_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
				{
					rs2_vector accel_data = motion_frame.get_motion_data();
					rotation_estimate.process_accel(accel_data);
//					std::cout << "Got motion frame:\t\t\t(x, y, z) = " << "(" << accel_data.x << ", " << accel_data.y << ", " << accel_data.z << ")" << std::endl;
				}

/*
				float3 rotation = rotation_estimate.get_theta();
				std::cout << "(R, Y, P) = (" << rotation.x << ", " << rotation.y << ", " << rotation.z << ")" << std::endl;

				float3 second_rotation = rotation_estimate.rotation_matrix_to_ryp();
				std::cout << "(R, Y, P) = (" << second_rotation.x << ", " << second_rotation.y << ", " << second_rotation.z << ") second" << std::endl;
*/
			}
			else
			{
				// get a depth frame
				rs2::depth_frame depth = frames.get_depth_frame();
				rs2::video_frame color = frames.get_color_frame();

				uint16_t* depth_data = (uint16_t*)depth.get_data();
				float depth_at_edge = depth_data[0] * depth_scale;

				// create opencv matrix representations of the images
				cv::Mat depth_image(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
				cv::Mat color_image(cv::Size(COLOR_WIDTH, COLOR_HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

				depth_image.convertTo(depth_image, CV_64F);

				// transform images to align them with "down" -> 0 pitch, 0 roll
				cv::Mat rotation_matrix = cv::Mat::zeros( 3, 3, CV_64FC1 );
				rotation_estimate.get_rotation_matrix( rotation_matrix );

#if TRANSFORM
				cv::Mat transformed_image = depth_image.clone();
//				cv::warpPerspective( depth_image, transformed_image, rotation_matrix, depth_image.size(), cv::WARP_INVERSE_MAP );
				cv::warpPerspective( depth_image, transformed_image, rotation_matrix, depth_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT );
				depth_image = transformed_image.clone();
#endif

#if VISUALIZE
				// colorize the depth image (for display only)
				cv::Mat depth_image_colorized(depth_image);
				cv::convertScaleAbs(depth_image_colorized, depth_image_colorized, 0.03);
				cv::applyColorMap(depth_image_colorized, depth_image_colorized, cv::COLORMAP_JET);
				
				// display the images on the windows
				cv::imshow(depth_window_name, depth_image_colorized);
				cv::imshow(color_window_name, color_image);

				// wait a bit so the images actually appear
				cv::waitKey(10);
#endif
			}
		}
	}

	return 0;
}
