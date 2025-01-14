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



#ifndef OV_INIT_DYNAMICINITIALIZER_H
#define OV_INIT_DYNAMICINITIALIZER_H

#include "ceres/Factor_GenericPrior.h"
#include "ceres/Factor_ImageReprojCalib.h"
#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "init/InertialInitializerOptions.h"
#include "utils/helper.h"

#include "cpi/CpiV1.h"
#include "feat/FeatureHelper.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace ov_init {

class DynamicInitializer {
public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param imu_data_ Shared pointer to our IMU vector of historical information
   */
  explicit DynamicInitializer(
      const InertialInitializerOptions &params_,
      std::shared_ptr<ov_core::FeatureDatabase> db,
      std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief Try to get the initialized system
   *
   * @param[out] timestamp Timestamp we have initialized the state at (last imu
   * state)
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param _imu Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg,
   * ba)
   * @param _clones_IMU Map between imaging times and clone poses (q_GtoIi,
   * p_IiinG)
   * @param _features_SLAM Our current set of SLAM features (3d positions)
   * @param _calib_IMUtoCAM Calibration poses for each camera (R_ItoC, p_IinC)
   * @param _cam_intrinsics Camera intrinsics
   * @return True if we have successfully initialized our system
   */
  bool
  initialize(double &timestamp, Eigen::MatrixXd &covariance,
             std::vector<std::shared_ptr<ov_type::Type>> &order,
             std::shared_ptr<ov_type::IMU> &_imu,
             std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
             std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>
                 &_features_SLAM,
             std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>>
                 &_calib_IMUtoCAM,
             std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>>
                 &_cam_intrinsics);

private:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_DYNAMICINITIALIZER_H
