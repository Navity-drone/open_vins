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



#ifndef OV_INIT_INERTIALINITIALIZER_H
#define OV_INIT_INERTIALINITIALIZER_H

#include "dynamic/DynamicInitializer.h"
#include "init/InertialInitializerOptions.h"
#include "static/StaticInitializer.h"

#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace ov_init {

class InertialInitializer {

public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   */
  explicit InertialInitializer(InertialInitializerOptions &params_,
                               std::shared_ptr<ov_core::FeatureDatabase> db);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Append it to our vector
    imu_data->emplace_back(message);

    // Sort our imu data (handles any out of order measurements)
    // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const
    // IMUDATA j) {
    //    return i.timestamp < j.timestamp;
    //});

    // Loop through and delete imu messages that are older than our requested
    // time
    if (oldest_time != -1) {
      auto it0 = imu_data->begin();
      while (it0 != imu_data->end()) {
        if (message.timestamp < oldest_time) {
          it0 = imu_data->erase(it0);
        } else {
          it0++;
        }
      }
    }
  }

  /**
   * @brief Try to get the initialized system
   *
   *
   * @m_class{m-note m-warning}
   *
   * @par Processing Cost
   * This is a serial process that can take on orders of seconds to complete.
   * If you are a real-time application then you will likely want to call this
   * from a async thread which allows for this to process in the background. The
   * features used are cloned from the feature database thus should be
   * thread-safe to continue to append new feature tracks to the database.
   *
   * @param[out] timestamp Timestamp we have initialized the state at
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param[out] t_imu Our imu type (need to have correct ids)
   * @param wait_for_jerk If true we will wait for a "jerk"
   * @return True if we have successfully initialized our system
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance,
                  std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu,
                  bool wait_for_jerk = true);

protected:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /// Static initialization helper class
  std::shared_ptr<StaticInitializer> init_static;

  /// Dynamic initialization helper class
  std::shared_ptr<DynamicInitializer> init_dynamic;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
