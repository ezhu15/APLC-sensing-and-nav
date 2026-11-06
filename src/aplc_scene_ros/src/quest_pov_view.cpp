#include <OgreSceneNode.h>
#include <OgreCamera.h>
#include <OgreVector.h>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp"

namespace aplc_scene_ros
{
// FPS normally tracks only translation. Attach its local +X-forward camera
// basis to the complete headset frame, preserving yaw, pitch and roll.
class QuestPOV : public rviz_default_plugins::view_controllers::FPSViewController
{
public:
  QuestPOV()
  {
    resetOffsets();
    fov_ = new rviz_common::properties::FloatProperty(
      "Vertical FOV", 75.0f, "Vertical camera field of view in degrees.", this);
    fov_->setMin(30.0f); fov_->setMax(140.0f);
  }
  void update(float dt, float ros_dt) override
  {
    FPSViewController::update(dt, ros_dt);
    camera_->setFOVy(Ogre::Degree(fov_->getFloat()));
  }
  void reset() override { resetOffsets(); updateCamera(); }
  void handleMouseEvent(rviz_common::ViewportMouseEvent &) override {}
protected:
  void updateTargetSceneNode() override
  {
    if (target_scene_node_ && getNewTransform()) {
      target_scene_node_->setPosition(reference_position_);
      target_scene_node_->setOrientation(reference_orientation_);
      context_->queueRender();
    }
  }
private:
  rviz_common::properties::FloatProperty * fov_;
  void resetOffsets()
  {
    position_property_->setVector(Ogre::Vector3::ZERO);
    yaw_property_->setFloat(0.0f);
    pitch_property_->setFloat(0.0f);
  }
};
}
PLUGINLIB_EXPORT_CLASS(aplc_scene_ros::QuestPOV, rviz_common::ViewController)
