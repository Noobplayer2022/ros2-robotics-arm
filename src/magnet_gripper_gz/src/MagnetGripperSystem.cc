#include "magnet_gripper_gz/MagnetGripperSystem.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/DetachableJoint.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Joint.hh>

using namespace magnet_gripper_gz;
using namespace gz::sim;

MagnetGripperSystem::MagnetGripperSystem() = default;

void MagnetGripperSystem::Configure(
  const Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  EntityComponentManager & _ecm,
  EventManager &)
{
  modelEntity_ = _entity;
  Model model(_entity);

  if (_sdf->HasElement("link_name")) {
    gripperLinkName_ = _sdf->Get<std::string>("link_name");
  }
  if (_sdf->HasElement("max_distance")) {
    maxDistance_ = _sdf->Get<double>("max_distance");
  }
  if (_sdf->HasElement("attach_topic")) {
    attachTopic_ = _sdf->Get<std::string>("attach_topic");
  }
  if (_sdf->HasElement("status_topic")) {
    statusTopic_ = _sdf->Get<std::string>("status_topic");
  }

  gripperLink_ = model.LinkByName(_ecm, gripperLinkName_);
  if (gripperLink_ == kNullEntity) {
    gzerr << "MagnetGripper: link '" << gripperLinkName_
          << "' not found on model\n";
    return;
  }

  // gz-transport (ROS2 از طریق bridge به همین تاپیک‌ها وصل می‌شود)
  statusPub_ = node_.Advertise<gz::msgs::Boolean>(statusTopic_);
  node_.Subscribe(attachTopic_, &MagnetGripperSystem::OnAttachMsg, this);

  gzmsg << "MagnetGripper configured | link=" << gripperLinkName_
        << " max_distance=" << maxDistance_
        << " attach=" << attachTopic_
        << " status=" << statusTopic_ << "\n";

  PublishStatus(false);
}

void MagnetGripperSystem::OnAttachMsg(const gz::msgs::Boolean & _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  wantAttach_ = _msg.data();
  requestPending_ = true;
}

void MagnetGripperSystem::PublishStatus(bool _attached)
{
  gz::msgs::Boolean msg;
  msg.set_data(_attached);
  statusPub_.Publish(msg);
}

void MagnetGripperSystem::PreUpdate(
  const UpdateInfo & _info,
  EntityComponentManager & _ecm)
{
  if (_info.paused) {
    return;
  }

  bool doAttach = false;
  bool doDetach = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (requestPending_) {
      if (wantAttach_ && !attached_) {
        doAttach = true;
      } else if (!wantAttach_ && attached_) {
        doDetach = true;
      }
      requestPending_ = false;
    }
  }

  if (doAttach) {
    TryAttach(_ecm);
  }
  if (doDetach) {
    Detach(_ecm);
  }
}

void MagnetGripperSystem::TryAttach(EntityComponentManager & _ecm)
{
  if (gripperLink_ == kNullEntity) {
    return;
  }

  const auto gripperPose = worldPose(gripperLink_, _ecm);
  const auto gripperPos = gripperPose.Pos();

  Entity bestLink = kNullEntity;
  double bestDist = maxDistance_;

  // همه مدل‌ها به‌جز خود ربات
  _ecm.Each<components::Model, components::Name>(
    [&](const Entity & modelEnt,
        const components::Model *,
        const components::Name * name) -> bool
    {
      if (modelEnt == modelEntity_) {
        return true;
      }
      // مکعب / اشیاء قابل برداشتن
      Model other(modelEnt);
      for (const Entity linkEnt : other.Links(_ecm)) {
        const auto pose = worldPose(linkEnt, _ecm);
        const double d = (pose.Pos() - gripperPos).Length();
        if (d < bestDist) {
          bestDist = d;
          bestLink = linkEnt;
        }
      }
      return true;
    });

  if (bestLink == kNullEntity) {
    gzwarn << "MagnetGripper: no object within " << maxDistance_ << " m\n";
    PublishStatus(false);
    return;
  }

  // ساخت joint ثابت جداشدنی بین gripper و object
  jointEntity_ = _ecm.CreateEntity();
  _ecm.CreateComponent(
    jointEntity_,
    components::DetachableJoint({gripperLink_, bestLink, "fixed"}));

  attachedChildLink_ = bestLink;
  attached_ = true;

  std::string childName = "unknown";
  if (auto * n = _ecm.Component<components::Name>(bestLink)) {
    childName = n->Data();
  }
  gzmsg << "MagnetGripper: ATTACHED to link '" << childName
        << "' (d=" << bestDist << " m)\n";
  PublishStatus(true);
}

void MagnetGripperSystem::Detach(EntityComponentManager & _ecm)
{
  if (!attached_) {
    PublishStatus(false);
    return;
  }

  if (jointEntity_ != kNullEntity) {
    _ecm.RequestRemoveEntity(jointEntity_);
    jointEntity_ = kNullEntity;
  }
  attachedChildLink_ = kNullEntity;
  attached_ = false;

  gzmsg << "MagnetGripper: DETACHED\n";
  PublishStatus(false);
}

GZ_ADD_PLUGIN(
  magnet_gripper_gz::MagnetGripperSystem,
  gz::sim::System,
  magnet_gripper_gz::MagnetGripperSystem::ISystemConfigure,
  magnet_gripper_gz::MagnetGripperSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  magnet_gripper_gz::MagnetGripperSystem,
  "magnet_gripper_gz::MagnetGripperSystem")