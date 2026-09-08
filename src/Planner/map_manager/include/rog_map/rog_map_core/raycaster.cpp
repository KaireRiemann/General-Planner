/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#include "raycaster.h"

namespace rog_map {
    namespace raycaster {
        RayCaster::RayCaster(const double &resolution) {
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
            throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif

#ifndef ORIGIN_AT_CORNER
#ifndef ORIGIN_AT_CENTER
            throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both false!");
#endif
#endif

            if (!std::isfinite(resolution) || resolution <= 0) {
                throw std::runtime_error(" -- [Raycaster]: resolution must be positive!");
            } else {
                resolution_ = resolution;
            }
        }

        void RayCaster::setResolution(const double &resolution) {
            if (!std::isfinite(resolution) || resolution <= 0) {
                throw std::invalid_argument("RayCaster resolution must be finite and positive");
            }
            valid_input_ = false;
            resolution_ = resolution;
        }

        void RayCaster::posToIndex(const double d, int &id) const {
#ifdef ORIGIN_AT_CORNER
            id = std::floor((d / resolution_));
#endif

#ifdef ORIGIN_AT_CENTER
            id = static_cast<int>((d / resolution_ + signum(d) * 0.5));
#endif
        }

        void RayCaster::indexToPos(const int &id, double &d) const {
#ifdef ORIGIN_AT_CORNER
            d = (id + 0.5) * resolution_;
#endif
#ifdef ORIGIN_AT_CENTER
            d = id * resolution_;
#endif
        }

        bool RayCaster::setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
            valid_input_ = false;
            remaining_steps_ = 0;
            if (!std::isfinite(resolution_) || resolution_ <= 0) {
                throw std::runtime_error(" -- [RayCaster] Resolution is not set!");
            }
            // Reject invalid/out-of-range coordinates before floating-to-int
            // conversion. A reused caster must not retain its previous ray.
            const double index_limit = std::numeric_limits<int>::max() / 4.0;
            if (!start.allFinite() || !end.allFinite() ||
                (start / resolution_).cwiseAbs().maxCoeff() > index_limit ||
                (end / resolution_).cwiseAbs().maxCoeff() > index_limit) {
                return false;
            }
            start_x_d_ = start.x();
            start_y_d_ = start.y();
            start_z_d_ = start.z();

            end_x_d_ = end.x();
            end_y_d_ = end.y();
            end_z_d_ = end.z();

            // Calculate expand dir and index
            posToIndex(start_x_d_, start_x_i_);
            posToIndex(start_y_d_, start_y_i_);
            posToIndex(start_z_d_, start_z_i_);

            posToIndex(end_x_d_, end_x_i_);
            posToIndex(end_y_d_, end_y_i_);
            posToIndex(end_z_d_, end_z_i_);

            int delta_X = end_x_i_ - start_x_i_;
            int delta_Y = end_y_i_ - start_y_i_;
            int delta_Z = end_z_i_ - start_z_i_;
            remaining_steps_ = static_cast<std::int64_t>(std::abs(delta_X)) +
                               std::abs(delta_Y) + std::abs(delta_Z);
            valid_input_ = true;

            cur_ray_pt_id_x_ = start_x_i_;
            cur_ray_pt_id_y_ = start_y_i_;
            cur_ray_pt_id_z_ = start_z_i_;

            expand_dir_x_ = static_cast<int>(signum(static_cast<int>(delta_X)));
            expand_dir_y_ = static_cast<int>(signum(static_cast<int>(delta_Y)));
            expand_dir_z_ = static_cast<int>(signum(static_cast<int>(delta_Z)));
//                std::cout<<"expand_dir_x_:"<<expand_dir_x_<<" expand_dir_y_:"<<expand_dir_y_<<" expand_dir_z_:"<<expand_dir_z_<<std::endl;
            if (expand_dir_x_ == 0 && expand_dir_y_ == 0 && expand_dir_z_ == 0) {
                return false;
            }

            double dis_x_over_t, dis_y_over_t, dis_z_over_t, t_max;
            dis_x_over_t = std::abs(end_x_d_ - start_x_d_);
            dis_y_over_t = std::abs(end_y_d_ - start_y_d_);
            dis_z_over_t = std::abs(end_z_d_ - start_z_d_);

            t_max = sqrt(dis_x_over_t * dis_x_over_t + dis_y_over_t * dis_y_over_t + dis_z_over_t * dis_z_over_t);

            dis_x_over_t /= t_max;
            dis_y_over_t /= t_max;
            dis_z_over_t /= t_max;

            // construct the look-up table
            // Which means, how long is t change when extend one resolution along A axis
            t_when_step_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_x_over_t);
            t_when_step_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_y_over_t);
            t_when_step_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() :
                             std::abs(resolution_ / dis_z_over_t);

            // Calculate nex bound
            double start_grid_center_x, start_grid_center_y, start_grid_center_z;
            indexToPos(start_x_i_, start_grid_center_x);
            indexToPos(start_y_i_, start_grid_center_y);
            indexToPos(start_z_i_, start_grid_center_z);
            double next_bound_x, next_bound_y, next_bound_z;
            next_bound_x = start_grid_center_x + expand_dir_x_ * resolution_ * 0.5;
            next_bound_y = start_grid_center_y + expand_dir_y_ * resolution_ * 0.5;
            next_bound_z = start_grid_center_z + expand_dir_z_ * resolution_ * 0.5;


            t_to_bound_x_ = expand_dir_x_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_x - start_x_d_) / dis_x_over_t;
            t_to_bound_y_ = expand_dir_y_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_y - start_y_d_) / dis_y_over_t;
            t_to_bound_z_ = expand_dir_z_ == 0 ? std::numeric_limits<double>::max() :
                            std::fabs(next_bound_z - start_z_d_) / dis_z_over_t;


//                std::cout << "t_to_bound_x_: " << t_to_bound_x_ << " t_to_bound_y_: " << t_to_bound_y_
//                          << " t_to_bound_z_: " << t_to_bound_z_ << std::endl;

            step_num_ = 0;
            first_point = true;
            return true;
        }

        bool RayCaster::step(Eigen::Vector3d &ray_pt) {
            if (!valid_input_ || remaining_steps_ <= 0) {
                return false;
            }
            --remaining_steps_;
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
            throw std::runtime_error(" -- [RayCaster]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif

#ifndef ORIGIN_AT_CORNER
#ifndef ORIGIN_AT_CENTER
            throw std::runtime_error(" -- [RayCaster]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both false!");
#endif
#endif
            step_num_++;
//                std::cout << "Iter " << step_num_ << "========================================" << std::endl;
//                std::cout << "start: " << start_x_d_ << ", " << start_y_d_ << ", " << start_z_d_ << std::endl;
//                std::cout << "end: " << end_x_d_ << ", " << end_y_d_ << ", " << end_z_d_ << std::endl;
//                std::cout << "end_id: " << end_x_i_ << ", " << end_y_i_ << ", " << end_z_i_ << std::endl;
//                std::cout << "cur_id: " << cur_ray_pt_id_x_ << ", " << cur_ray_pt_id_y_ << ", " << cur_ray_pt_id_z_
//                          << std::endl;
//                std::cout << "t_to_bound
//                : " << t_to_bound_x_ << ", " << t_to_bound_y_ << ", " << t_to_bound_z_
//                          << std::endl;
            // shape the output
            indexToPos(cur_ray_pt_id_x_, ray_pt.x());
            indexToPos(cur_ray_pt_id_y_, ray_pt.y());
            indexToPos(cur_ray_pt_id_z_, ray_pt.z());
            if (cur_ray_pt_id_x_ == end_x_i_ && cur_ray_pt_id_y_ == end_y_i_ && cur_ray_pt_id_z_ == end_z_i_) {
                return false;
            }

            // compute firest bound point
            // Once an axis reaches the target voxel it must never advance
            // again. Boundary ties/roundoff (especially at negative grid
            // coordinates) otherwise allow overshoot and a non-terminating ray.
            if (cur_ray_pt_id_x_ == end_x_i_) t_to_bound_x_ = std::numeric_limits<double>::infinity();
            if (cur_ray_pt_id_y_ == end_y_i_) t_to_bound_y_ = std::numeric_limits<double>::infinity();
            if (cur_ray_pt_id_z_ == end_z_i_) t_to_bound_z_ = std::numeric_limits<double>::infinity();
            if (t_to_bound_x_ < t_to_bound_y_) {
                if (t_to_bound_x_ < t_to_bound_z_) {
//                        std::cout << "Expand X" << std::endl;
                    cur_ray_pt_id_x_ += expand_dir_x_;
                    t_to_bound_x_ += t_when_step_x_;
                } else {
//                        std::cout << "Expand Z" << std::endl;
                    cur_ray_pt_id_z_ += expand_dir_z_;
                    t_to_bound_z_ += t_when_step_z_;
                }
            } else {
                if (t_to_bound_y_ < t_to_bound_z_) {
//                        std::cout << "Expand Y" << std::endl;
                    cur_ray_pt_id_y_ += expand_dir_y_;
                    t_to_bound_y_ += t_when_step_y_;
                } else {
//                        std::cout << "Expand Z" << std::endl;
                    cur_ray_pt_id_z_ += expand_dir_z_;
                    t_to_bound_z_ += t_when_step_z_;

                }
            }
            return true;
        }

    }
}
