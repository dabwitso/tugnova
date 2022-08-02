#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>
#include <std_msgs/Int16.h>
#include <carctl_msgs/emergency_status.h>

/**
 * チェックリストデータ構造.
 */
class CheckListData {
  public:
    CheckListData(const std::string name, const int status) {
      this->name = name;
      this->status = status;
    }

    std::string getName() { return name; }
    int getStatus() { return status; }
    void setStatus(int status) { this->status = status; }

    typedef std::shared_ptr<CheckListData> Ptr;
  private:
    std::string name;
    int status;
};

std::vector<CheckListData::Ptr> error_list;
const int RUN = 0;
const int STOP = 1;

/** チェックリストへの登録. */
void insertList(const std::string name, const int status) {
  CheckListData::Ptr data(new CheckListData(name, status));
  error_list.push_back(data);
}

/** チェックリストの追加と更新. */
void emergencyErrorCallback(const carctl_msgs::emergency_status::ConstPtr& msg) {
  std::string name = msg->service_name;
  int status = msg->status;

  if (error_list.empty()) { insertList(name, status); }

  std::vector<CheckListData::Ptr>::const_iterator target =
  std::find_if(error_list.begin(), error_list.end(), [&name](CheckListData::Ptr& data) {
    return (data->getName() == name);
  });

  if (target == error_list.end()) {
    insertList(name, status);
  } else {
    (*target)->setStatus(status);
  }
}

/** コマンド送信. */
void sendCommand(ros::Publisher& pub) {
  if (error_list.empty()) {
    pub.publish(RUN);
    return;
  }

  std::vector<CheckListData::Ptr>::const_iterator target =
  std::find_if(error_list.begin(), error_list.end(), [](CheckListData::Ptr& data) {
    return (data->getStatus() == 1/*= ERROR*/);
  });

  int command = STOP;
  if (target == error_list.end()) { command = RUN; }
  pub.publish(command);
}

/** メイン. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_stop");
  ros::NodeHandle n;

  error_list.clear();
  ros::Publisher pub_emergency_stop_state = n.advertise<std_msgs::Int16>("emergency_stop_state", 10);
  ros::Subscriber sub_emergency_error = n.subscribe("emergency_error", 10, emergencyErrorCallback);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    sendCommand(pub_emergency_stop_state);
    loop_rate.sleep();
  }
}
