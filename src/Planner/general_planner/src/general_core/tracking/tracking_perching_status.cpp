/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

namespace general_planner {
    namespace {
        bool trackingPerchingPerchingStatus(
                const TrackingPerchingTransitionManager::Status status) {
            return status == TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
                   status == TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT;
        }
    }

    bool GeneralPlanner::trackingPerchingPerchingActive() const {
        return tracking_perching_manager_ &&
               trackingPerchingPerchingStatus(tracking_perching_manager_->status());
    }

    bool GeneralPlanner::trackingPerchingContactReached() const {
        return tracking_perching_manager_ &&
               tracking_perching_manager_->status() ==
                   TrackingPerchingTransitionManager::Status::CONTACT;
    }

    TrackingPerchingTransitionManager::Status GeneralPlanner::trackingPerchingStatus() const {
        return tracking_perching_manager_
                   ? tracking_perching_manager_->status()
                   : TrackingPerchingTransitionManager::Status::TRACKING_ONLY;
    }

    void GeneralPlanner::markTrackingPerchingContact() {
        if (tracking_perching_manager_ &&
            trackingPerchingPerchingStatus(tracking_perching_manager_->status())) {
            tracking_perching_manager_->onContact();
        }
    }

}
