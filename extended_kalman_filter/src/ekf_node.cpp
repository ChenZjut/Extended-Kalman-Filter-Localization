/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2022/1/7 上午11:02
 * @LastEditors: ubuntu
 * @LastEditTime: 2022/1/7 上午11:02
 * @Version 1.0
 */

#include <extended_kalman_filter/ekf.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_test_node");
    EKFNS::EKF app;
    app.run();
    return 0;
}