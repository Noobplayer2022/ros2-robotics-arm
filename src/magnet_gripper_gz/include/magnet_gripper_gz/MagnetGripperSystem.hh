#ifndef MAGNET_GRIPPER_GZ__MAGNET_GRIPPER_SYSTEM_HH_
#define MAGNET_GRIPPER_GZ__MAGNET_GRIPPER_SYSTEM_HH_

#include <memory>
#include <mutex>
#include <string>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

namespace magnet_gripper_gz
{

class MagnetGripperSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  MagnetGripperSystem();
  ~MagnetGripperSystem() override = default;

  void Configure(
      const gz::sim::Entity & _entity,
      const std::shared_ptr<const sdf::Element> & _sdf,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo & _info,
      gz::sim::EntityComponentManager & _ecm) override;

private:
  void OnAttachMsg(const gz::msgs::Boolean & _msg);
  void PublishStatus(bool _attached);
  void TryAttach(gz::sim::EntityComponentManager & _ecm);
  void Detach(gz::sim::EntityComponentManager & _ecm);

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity gripperLink_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};
  gz::sim::Entity attachedChildLink_{gz::sim::kNullEntity};

  std::string gripperLinkName_{"link_5"};
  std::string attachTopic_{"/magnet/attach"};
  std::string statusTopic_{"/magnet/status"};
  double maxDistance_{0.08};

  bool wantAttach_{false};
  bool attached_{false};
  bool requestPending_{false};

  std::mutex mutex_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher statusPub_;
};

}  // namespace magnet_gripper_gz

#endif