#pragma once

#include <vector>

#include <OgreVector3.h>
#include <rviz/tool.h>

namespace rviz {
    class Arrow;
    class ViewportMouseEvent;
}

namespace general_planner_rviz_plugins {

    class Pose3DTool : public rviz::Tool {
    public:
        Pose3DTool();
        ~Pose3DTool() override;

        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz::ViewportMouseEvent &event) override;

    protected:
        virtual void onPoseSet(double x,
                               double y,
                               double z,
                               double yaw,
                               bool height_selected) = 0;

    private:
        enum class State {
            POSITION,
            ORIENTATION,
            HEIGHT,
        };

        void clearHeightMarkers();

        State state_{State::POSITION};
        rviz::Arrow *arrow_{nullptr};
        std::vector<rviz::Arrow *> height_markers_;
        Ogre::Vector3 position_{Ogre::Vector3::ZERO};
        double initial_z_{0.0};
        double previous_mouse_y_{0.0};
        double yaw_{0.0};
        bool height_selected_{false};
    };

} // namespace general_planner_rviz_plugins
