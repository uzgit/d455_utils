// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

#pragma once

#include <cmath>  // sqrt, sqrtf
#include "example.hpp"

namespace rs2 {


struct float3
{
    float x, y, z;

    float length() const { return sqrt( x * x + y * y + z * z ); }

    float3 normalize() const { return ( length() > 0 ) ? float3{ x / length(), y / length(), z / length() } : *this; }
};

inline float3 cross( const float3 & a, const float3 & b )
{
    return { a.y * b.z - b.y * a.z, a.x * b.z - b.x * a.z, a.x * b.y - a.y * b.x };
}

inline float3 operator*( const float3 & a, float t )
{
    return { a.x * t, a.y * t, a.z * t };
}

inline float3 operator/( const float3 & a, float t )
{
    return { a.x / t, a.y / t, a.z / t };
}

inline float3 operator+( const float3 & a, const float3 & b )
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline float3 operator-( const float3 & a, const float3 & b )
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

}  // namespace rs2


class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
//    float alpha = 0.98f;
    float alpha = 0.5f; // this is a test, but so far seems to allow it to react more quickly without causing problems
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(-accel_data.y, -accel_data.z);  // switched the signs on both of these numbers to orient it so z=0 means the camera is facing down
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI_FL;
        }
        else
        {
            /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }
    
    // Returns the current rotation angle
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
   
    void get_rotation_matrix(cv::Mat& destination)
    {
	float3 local_theta = get_theta();
	double roll = local_theta.x;
	double yaw  = 0; // do not use the yaw
	double pitch = local_theta.z;

/*
	roll  *= 0.0005;
	pitch *= 0.0005;
	yaw = 0;
*/

        //Precompute sines and cosines of Euler angles
        double su = sin(roll);
        double cu = cos(roll);
        double sv = sin(pitch);
        double cv = cos(pitch);
        double sw = sin(yaw);
        double cw = cos(yaw);

        destination.at<double>(0, 0) = cv*cw;
        destination.at<double>(0, 1) = su*sv*cw - cu*sw;
        destination.at<double>(0, 2) = su*sw + cu*sv*cw;
        destination.at<double>(1, 0) = cv*sw;
        destination.at<double>(1, 1) = cu*cw + su*sv*sw;
        destination.at<double>(1, 2) = cu*sv*sw - su*cw;
        destination.at<double>(2, 0) = -sv;
        destination.at<double>(2, 1) = su*cv;
        destination.at<double>(2, 2) = cu*cv;   	

/*	
	for( int i = 0; i < destination.size().height; i ++ )
	{
		for( int ii = 0; ii < destination.size().width; ii ++)
		{
			std::cout << destination.at<double>(i, ii) << "\t";
		}
		std::cout << std::endl;
	}
*/
    }

    float3 rotation_matrix_to_ryp()
    {
	    float3 result;

	    cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
	    get_rotation_matrix(rotation_matrix);

	    result.z = asin( -rotation_matrix.at<double>(2,0) );
	    result.y = atan2( rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0) );
	    result.x = atan2( rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2) );

	    return result;
    }

/*    

    std::string type2str(int type) {
	    std::string r;

	  uchar depth = type & CV_MAT_DEPTH_MASK;
	  uchar chans = 1 + (type >> CV_CN_SHIFT);

	  switch ( depth ) {
	    case CV_8U:  r = "8U"; break;
	    case CV_8S:  r = "8S"; break;
	    case CV_16U: r = "16U"; break;
	    case CV_16S: r = "16S"; break;
	    case CV_32S: r = "32S"; break;
	    case CV_32F: r = "32F"; break;
	    case CV_64F: r = "64F"; break;
	    default:     r = "User"; break;
	  }

	  r += "C";
	  r += (chans+'0');

	  return r;
	}

    void get_rotation_matrix(cv::Mat& destination)
    {
	// transform images to align them with "down" -> 0 pitch, 0 roll
	cv::Point2f src_vertices[4];
	src_vertices[0] = cv::Point(20, 20);
	src_vertices[1] = cv::Point(300, 30);
	src_vertices[2] = cv::Point(15, 460);
	src_vertices[3] = cv::Point(620, 470);

	cv::Point2f dst_vertices[4];
	dst_vertices[0] = cv::Point(0, 0);
	dst_vertices[1] = cv::Point(639, 0); // Bug was: had mistakenly switched these 2 parameters
	dst_vertices[2] = cv::Point(0, 479);
	dst_vertices[3] = cv::Point(639, 479);

	cv::Mat transformation_matrix = cv::getPerspectiveTransform(dst_vertices, src_vertices);

	for( int i = 0; i < transformation_matrix.size().height; i ++ )
	{
		for( int ii = 0; ii < transformation_matrix.size().width; ii ++)
		{
			std::cout << transformation_matrix.at<double>(i, ii) << "\t";
			destination.at<double>(i, ii) = transformation_matrix.at<double>(i, ii);
		}
		std::cout << std::endl;
	}

	std::cout << "type 1: " << type2str(transformation_matrix.type()) << std::endl;
	std::cout << "type 2: " << type2str(destination.type()) << std::endl;

//	destination = transformation_matrix.clone();
	std::cout << "type 3: " << type2str(destination.type()) << std::endl;

	std::cout << "height: " << destination.size().height << ", width: " << destination.size().width << std::endl;

    }
*/

    /*
    void get_rotation_matrix(cv::Mat& destination)
    {
	destination.at<double>(0, 0) = 0.44;
	destination.at<double>(0, 1) = -0.027;
	destination.at<double>(0, 2) = 20;
	destination.at<double>(1, 0) = 0.16;
	destination.at<double>(1, 1) = 0.4;
	destination.at<double>(1, 2) = 20;
	destination.at<double>(2, 0) = 1.9e-05;
	destination.at<double>(2, 1) = -0.0011;
	destination.at<double>(2, 2) = 1;
	destination.at<double>(3, 0) = 0.44;
	destination.at<double>(3, 1) = -0.027;
	destination.at<double>(3, 2) = 20;
	destination.at<double>(4, 0) = 0.16;
	destination.at<double>(4, 1) = 0.4;
	destination.at<double>(4, 2) = 20;
	destination.at<double>(5, 0) = 1.9e-05;
	destination.at<double>(5, 1) = -0.0011;
	destination.at<double>(5, 2) = 1;
    }
    */
};


