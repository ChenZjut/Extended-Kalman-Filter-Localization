/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2022/1/7 上午11:04
 * @LastEditors: ubuntu
 * @LastEditTime: 2022/1/7 上午11:04
 * @Version 1.0
 */
#ifndef SRC_EKF_H
#define SRC_EKF_H

#define SIM_TIME 50.0
#define DT 0.1

#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

using namespace std;

namespace EKFNS
{
    class EKF{
    private:
        float time;

        // control input
        Eigen::Vector2f u;

        // noise control input
        Eigen::Vector2f ud;

        // observation z
        Eigen::Vector2f z;

        // dead reckoning
        Eigen::Vector4f xDR;

        // ground truth reading
        Eigen::Vector4f xTrue;

        // Estimation
        Eigen::Vector4f xEst;

        vector<Eigen::Vector4f> hxDR;
        vector<Eigen::Vector4f> hxTrue;
        vector<Eigen::Vector4f> hxEst;
        vector<Eigen::Vector2f> hz;

        Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u);
        Eigen::Matrix4f jacobF(Eigen::Vector4f x, Eigen::Vector2f u);

        Eigen::Vector2f observation_model(Eigen::Vector4f x);
        Eigen::Matrix<float, 2, 4> jacobH();

        void ekf_estimation(Eigen::Vector4f& xEst, Eigen::Matrix4f& PEst,
                            Eigen::Vector2f z, Eigen::Vector2f u,
                            Eigen::Matrix4f Q, Eigen::Matrix2f R);

        cv::Point2i cv_offset(Eigen::Vector2f e_p, int image_width=2000, int image_height=2000);

        void ellipse_drawing(cv::Mat bg_img, Eigen::Matrix2f pest, Eigen::Vector2f center,
                             cv::Scalar ellipse_color=cv::Scalar(0, 0, 255));

    public:
        EKF();
        ~EKF();
        void run();
    };
}


#endif //SRC_EKF_H
