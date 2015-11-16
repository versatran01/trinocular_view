#include <ros/ros.h>

namespace snapshot {

class SnapshotNode {
 public:
  SnapshotNode(const ros::NodeHandle& pnh);

 private:
  ros::NodeHandle pnh_;
};

SnapshotNode::SnapshotNode(const ros::NodeHandle& pnh) : pnh_(pnh) {
}

}  // namespace snapshot

int main(int argc, char** argv) {
  ros::init(argc, argv, "snapshot_node");
  ros::NodeHandle nh, pnh("~");
}
