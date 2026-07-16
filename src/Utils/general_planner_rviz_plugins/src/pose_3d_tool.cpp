/*
 * The mouse interaction follows the BSD-licensed EGO-Planner V2 3D goal tool:
 * choose XY/yaw with the left button, then hold right as well to adjust height.
 */

#include "pose_3d_tool.hpp"

#include <cmath>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <QEvent>

#include <rviz/geometry.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/viewport_mouse_event.h>

namespace general_planner_rviz_plugins {

    namespace {
        constexpr double kHeightPixelsPerMeter = 50.0;
        constexpr double kHeightMarkerInterval = 0.5;
    }

    Pose3DTool::Pose3DTool() = default;

    Pose3DTool::~Pose3DTool() {
        clearHeightMarkers();
        delete arrow_;
    }

    void Pose3DTool::onInitialize() {
        arrow_ = new rviz::Arrow(scene_manager_, nullptr, 2.0F, 0.2F, 0.5F, 0.35F);
        arrow_->setColor(0.0F, 1.0F, 0.0F, 1.0F);
        arrow_->getSceneNode()->setVisible(false);
    }

    void Pose3DTool::activate() {
        setStatus("Left-drag: XY/yaw. Hold left+right and move vertically: height.");
        state_ = State::POSITION;
        yaw_ = 0.0;
        height_selected_ = false;
    }

    void Pose3DTool::deactivate() {
        clearHeightMarkers();
        if (arrow_ != nullptr) {
            arrow_->getSceneNode()->setVisible(false);
        }
    }

    void Pose3DTool::clearHeightMarkers() {
        for (rviz::Arrow *marker: height_markers_) {
            delete marker;
        }
        height_markers_.clear();
    }

    int Pose3DTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
        int flags = 0;
        const Ogre::Quaternion arrow_plane_rotation(
                Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

        if (event.leftDown()) {
            if (state_ != State::POSITION) {
                return flags;
            }
            Ogre::Vector3 intersection;
            Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0F);
            if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane,
                                                  event.x, event.y, intersection)) {
                position_ = intersection;
                initial_z_ = position_.z;
                previous_mouse_y_ = event.y;
                yaw_ = 0.0;
                height_selected_ = false;
                arrow_->setPosition(position_);
                state_ = State::ORIENTATION;
                flags |= Render;
            }
        } else if (event.type == QEvent::MouseMove && event.left()) {
            if (state_ == State::ORIENTATION) {
                Ogre::Vector3 cursor_position;
                Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0F);
                if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane,
                                                      event.x, event.y, cursor_position)) {
                    yaw_ = std::atan2(cursor_position.y - position_.y,
                                      cursor_position.x - position_.x);
                    arrow_->getSceneNode()->setVisible(true);
                    arrow_->setOrientation(
                            Ogre::Quaternion(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z) *
                            arrow_plane_rotation);
                    initial_z_ = position_.z;
                    previous_mouse_y_ = event.y;
                    if (event.right()) {
                        state_ = State::HEIGHT;
                        height_selected_ = true;
                    }
                    flags |= Render;
                }
            }

            if (state_ == State::HEIGHT) {
                const double current_mouse_y = event.y;
                const double delta_y = current_mouse_y - previous_mouse_y_;
                previous_mouse_y_ = current_mouse_y;
                position_.z -= delta_y / kHeightPixelsPerMeter;
                arrow_->setPosition(position_);

                clearHeightMarkers();
                const int marker_count = static_cast<int>(
                        std::ceil(std::abs(initial_z_ - position_.z) /
                                  kHeightMarkerInterval));
                for (int i = 0; i < marker_count; ++i) {
                    auto *marker = new rviz::Arrow(scene_manager_, nullptr,
                                                   0.5F, 0.1F, 0.0F, 0.1F);
                    marker->setColor(0.0F, 1.0F, 0.0F, 1.0F);
                    marker->getSceneNode()->setVisible(true);
                    Ogre::Vector3 marker_position = position_;
                    const double direction = initial_z_ - position_.z > 0.0 ? 1.0 : -1.0;
                    marker_position.z = initial_z_ - direction * i * kHeightMarkerInterval;
                    marker->setPosition(marker_position);
                    marker->setOrientation(
                            Ogre::Quaternion(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z) *
                            arrow_plane_rotation);
                    height_markers_.push_back(marker);
                }
                flags |= Render;
            }
        } else if (event.leftUp() &&
                   (state_ == State::ORIENTATION || state_ == State::HEIGHT)) {
            clearHeightMarkers();
            onPoseSet(position_.x, position_.y, position_.z, yaw_, height_selected_);
            flags |= Finished | Render;
        }

        return flags;
    }

} // namespace general_planner_rviz_plugins
