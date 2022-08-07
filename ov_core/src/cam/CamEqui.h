/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */



#ifndef OV_CORE_CAM_EQUI_H
#define OV_CORE_CAM_EQUI_H

#include "CamBase.h"

namespace ov_core {

class CamEqui : public CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamEqui(int width, int height) : CamBase(width, height) {}

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera
   * matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {

    // Determine what camera parameters we should use
    cv::Matx33d camK = camera_k_OPENCV;
    cv::Vec4d camD = camera_d_OPENCV;

    // Convert point to opencv format
    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv_dist(0);
    mat.at<float>(0, 1) = uv_dist(1);
    mat = mat.reshape(2); // Nx1, 2-channel

    // Undistort it!
    cv::fisheye::undistortPoints(mat, mat, camK, camD);

    // Construct our return vector
    Eigen::Vector2f pt_out;
    mat = mat.reshape(1); // Nx2, 1-channel
    pt_out(0) = mat.at<float>(0, 0);
    pt_out(1) = mat.at<float>(0, 1);
    return pt_out;
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw
   * image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {

    // Get our camera parameters
    Eigen::MatrixXd cam_d = camera_values;

    // Calculate distorted coordinates for fisheye
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double theta = std::atan(r);
    double theta_d =
        theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) +
        cam_d(6) * std::pow(theta, 7) + cam_d(7) * std::pow(theta, 9);

    // Handle when r is small (meaning our xy is near the camera center)
    double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    // Calculate distorted coordinates for fisheye
    Eigen::Vector2f uv_dist;
    double x1 = uv_norm(0) * cdist;
    double y1 = uv_norm(1) * cdist;
    uv_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
    uv_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));
    return uv_dist;
  }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic
   * parameters
   */
  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm,
                                Eigen::MatrixXd &H_dz_dzn,
                                Eigen::MatrixXd &H_dz_dzeta) override {

    // Get our camera parameters
    Eigen::MatrixXd cam_d = camera_values;

    // Calculate distorted coordinates for fisheye
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double theta = std::atan(r);
    double theta_d =
        theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) +
        cam_d(6) * std::pow(theta, 7) + cam_d(7) * std::pow(theta, 9);

    // Handle when r is small (meaning our xy is near the camera center)
    double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    // Jacobian of distorted pixel to "normalized" pixel
    Eigen::Matrix<double, 2, 2> duv_dxy = Eigen::Matrix<double, 2, 2>::Zero();
    duv_dxy << cam_d(0), 0, 0, cam_d(1);

    // Jacobian of "normalized" pixel to normalized pixel
    Eigen::Matrix<double, 2, 2> dxy_dxyn = Eigen::Matrix<double, 2, 2>::Zero();
    dxy_dxyn << theta_d * inv_r, 0, 0, theta_d * inv_r;

    // Jacobian of "normalized" pixel to r
    Eigen::Matrix<double, 2, 1> dxy_dr = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dr << -uv_norm(0) * theta_d * inv_r * inv_r,
        -uv_norm(1) * theta_d * inv_r * inv_r;

    // Jacobian of r pixel to normalized xy
    Eigen::Matrix<double, 1, 2> dr_dxyn = Eigen::Matrix<double, 1, 2>::Zero();
    dr_dxyn << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of "normalized" pixel to theta_d
    Eigen::Matrix<double, 2, 1> dxy_dthd = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dthd << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of theta_d to theta
    double dthd_dth = 1 + 3 * cam_d(4) * std::pow(theta, 2) +
                      5 * cam_d(5) * std::pow(theta, 4) +
                      7 * cam_d(6) * std::pow(theta, 6) +
                      9 * cam_d(7) * std::pow(theta, 8);

    // Jacobian of theta to r
    double dth_dr = 1 / (r * r + 1);

    // Total Jacobian wrt normalized pixel coordinates
    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    H_dz_dzn = duv_dxy *
               (dxy_dxyn + (dxy_dr + dxy_dthd * dthd_dth * dth_dr) * dr_dxyn);

    // Calculate distorted coordinates for fisheye
    double x1 = uv_norm(0) * cdist;
    double y1 = uv_norm(1) * cdist;

    // Compute the Jacobian in respect to the intrinsics
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    H_dz_dzeta(0, 0) = x1;
    H_dz_dzeta(0, 2) = 1;
    H_dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 3);
    H_dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 5);
    H_dz_dzeta(0, 6) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 7);
    H_dz_dzeta(0, 7) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 9);
    H_dz_dzeta(1, 1) = y1;
    H_dz_dzeta(1, 3) = 1;
    H_dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 3);
    H_dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 5);
    H_dz_dzeta(1, 6) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 7);
    H_dz_dzeta(1, 7) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 9);
  }
};

} // namespace ov_core

#endif /* OV_CORE_CAM_EQUI_H */