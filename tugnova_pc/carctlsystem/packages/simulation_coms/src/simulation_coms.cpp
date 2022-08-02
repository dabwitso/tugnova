#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include <udp_msgs/UdpSensorPacket.h>
#include <udp_msgs/UdpControlPacket.h>

ros::Publisher pub_sensor_packet;
ros::Subscriber sub_control_packet;
ros::Subscriber sub_autodrive;
ros::Subscriber sub_readydrive;

bool autodrive_flg = false;
bool readydrive_flg = false;

void contolpacketCallback(const udp_msgs::UdpControlPacket &msg)
{
  udp_msgs::UdpSensorPacket p;

  p.BrakePotVol = 0;
  p.AccelPotVol = 0;
  p.SteerPotVol = 0;
  p.tire_angle_rad = 0.0;
  p.ECU_vel = 0;
  p.R_Tire = 0;
  p.L_Tire = 0;
  p.SW = 0;
  p.Btry_V = 0.0;
  p.Btry_A = 0.0;
  p.SOC = 0;
  p.Inv_V = 0;
  p.Inv_A = 0;
  p.Motor_rpm = 0;
  p.Acc_trq = 0;
  p.encoder_speed_mps = 0.0;
  p.encoder_yawrate = 0.0;
  p.encoder_speedr_mps = 0.0;
  p.encoder_speedl_mps = 0.0;

  if (autodrive_flg == true)
  {
    if (msg.acc_vel_ref > 0.0)
    {
       p.ECU_vel = 1;
    }
    p.PLC_vel = msg.acc_vel_ref;
    p.ECUMode = 1;
  }
  else
  {
    p.ECU_vel = 0;
    p.PLC_vel = 0.0;
    p.ECUMode = 0;
  }

  if (readydrive_flg == true)
  {
    p.SOC = 1;
  }
  else
  {
    p.SOC = 0;
  }


  p.tire_angle_rad = msg.tir_ang_ref;
  p.SW = msg.wd_count;
  ROS_INFO("DEBUG: contolpacketCallback:PLC_vel:[%2.1lf]", p.PLC_vel * 3.6);
  pub_sensor_packet.publish(p);
  ros::spinOnce();
}

void autodriveCallback(const std_msgs::Int16 &msg)
{

  if (msg.data == 0)
  {
    autodrive_flg = false;
  }
  else if (msg.data == 1)
  {
    autodrive_flg = true;
  }
}

void readydriveCallback(const std_msgs::Int16 &msg)
{

  if (msg.data == 0)
  {
    readydrive_flg = false;
  }
  else if (msg.data == 1)
  {
    readydrive_flg = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation_coms");
  ros::NodeHandle nh;

  fprintf(stdout, "DEBUG: simulation_coms start.\n");
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    // pub
    pub_sensor_packet = nh.advertise<udp_msgs::UdpSensorPacket>("/plc_sensor_packet", 1);
    // sub
    sub_control_packet = nh.subscribe("/plc_control_packet", 1, contolpacketCallback);
    sub_autodrive = nh.subscribe("/plc_autodrive", 10, autodriveCallback);
    sub_readydrive = nh.subscribe("/plc_readydrive", 10, readydriveCallback);

    ros::spinOnce();
    loop_rate.sleep();
  }

  fprintf(stdout, "DEBUG: simulation_coms end.\n");
}
