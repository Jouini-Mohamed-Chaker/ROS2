#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/transport/Node.hh>

std::string topic_pub = "/cmd_vel";   // Publish to this topic
std::string topic_scan = "/scan";     // Publish scan data to this topic
ignition::transport::Node node;
auto pub = node.Advertise<ignition::msgs::Twist>(topic_pub);
auto scan_pub = node.Advertise<ignition::msgs::LaserScan>(topic_scan);

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Twist data;

  bool allMore = true;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < 1.0) 
    {
      allMore = false;
      break;
    }
  }
  if (allMore) // If all ranges are greater than one
  {
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
  }
  else
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.5);
  }
  pub.Publish(data);
  scan_pub.Publish(_msg); // Publish the scan data
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub = "/lidar";   // Subscribe to this topic
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}