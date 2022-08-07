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



namespace ov_core {}

/**
 * @namespace ov_type
 * @brief Dynamic type system types
 *
 * Types leveraged by the EKF system for covariance management.
 * These types store where they are in the covariance along with their current
 * estimate. Each also has an update function that takes a vector delta and
 * updates their manifold representation. Please see each type for details on
 * what they represent, but their names should be straightforward. See @ref
 * dev-index for high level details on how the type system and covariance
 * management works. Each type is described by the following:
 *
 * @code{.cpp}
 * class Type {
 * protected:
 *   // Current best estimate
 *   Eigen::MatrixXd _value;
 *   // Location of error state in covariance
 *   int _id = -1;
 *   // Dimension of error state
 *   int _size = -1;
 *   // Update eq. taking vector to their rep.
 *   void update(const Eigen::VectorXd dx);
 * };
 * @endcode
 *
 *
 */
namespace ov_type {}
