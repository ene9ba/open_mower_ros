// Info
// using lib from https://github.com/bmegli/wifi-scan

// things that must be installed on the system:

// WLAN-LIB
// apt-get update 
// apt-get install --yes libmnl0 libmnl-dev automake

// Ping
// apt-get update 
// apt-get install --yes iputils-ping

// rspi-gpio
// https://github.com/RPi-Distro/raspi-gpio
//  mkdir -p gitsrc  
//  cd gitsrc  
//  git clone https://github.com/RPi-Distro/raspi-gpio.git  
//	cd raspi-gpio 
//  ./configure 
//	make 
//  make install




#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
  

// includes Wifi Info Class
#include "./wifi_scan.h"
#include <stdio.h>  //printf
#include <unistd.h> //sleep


// includes CPU Info class  Temperature
#include <iostream>
#include <fstream> 
// includes CPU Info class  usage
#include <numeric>
#include <unistd.h>
#include <vector>


#include "mower_addons/raspi_inf.h"

#include "ros/ros.h"
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataDouble.h"

#define CPU_MIN_TEMP 65
#define CPU_MAX_TEMP 70


class Wifi_Quality {

  public:

      const char* wifi_channel = NULL;
      const char* target_ping = NULL;

      struct{
        std::string   wifi_mac      = "";     // MAC Address
        std::string   access_point  = "";     // accesspoint
        int           signal        = 0.0;      // current signal strength
        int           signal_avg    = 0;      // average signal strength
        int           rx_packets    = 0;      // received bytes
        int           tx_packets    = 0;      // tranceived bytes
        std::string   target_ping   = "";     // Target IP Address for Ping
        float         ping_response = 0;      // response Ping
      } network_info;
       
      Wifi_Quality() {
	      wifi_channel = getenv("OM_WIFI");
        if (wifi_channel != NULL) wifi=wifi_scan_init(wifi_channel);
        target_ping = getenv("OM_TARGET_PING");
        
                
      }


      bool is_connected() {
        if (wifi->command_channel.ifindex == 0)
          return false;
        else
          return true;
      }

      
      int get_station_info() {

          //get information from just the station we are associated with
		      //this is quick, you can call it at much faster frequency (e.g. 50 ms)
		      if (wifi_channel != NULL) {
            int status = wifi_scan_station(wifi, &station);
            if(status > 0) {
              network_info.wifi_mac        =     bssid_to_string(station.bssid, mac);            
              network_info.access_point    =     station.ssid;
              network_info.signal          =     (double)station.signal_dbm;
              network_info.signal_avg      =     station.signal_avg_dbm;
              network_info.rx_packets      =     station.rx_packets;
              network_info.tx_packets      =     station.tx_packets;
            }
            return status;
          }  

		
				

          return 0;
      }



      bool get_station_ping() {

          if(target_ping == NULL) return false;
          std::string read_string = "";

          char ch1[] = "ping -c1 ";
          char ch2[] = " > ";
          char logfilename[] = "ping.log";
          char* target = new char[std::strlen(ch1)+std::strlen(target_ping)+std::strlen(ch2)+std::strlen(logfilename)];
          std::strcpy(target,ch1);
          std::strcat(target,target_ping);
          std::strcat(target,ch2);
          std::strcat(target,logfilename);
          
          
          int ret_val = system(target);
	  
          
          
          try
          {
            char startstr1[] = "time";
            char startstr2[] = "Zeit";
            char endstr[] = "ms";

            std::ifstream pingfile(logfilename);
            getline(pingfile,read_string);
            getline(pingfile,read_string);
            pingfile.close();
            size_t start = read_string.find(startstr1);
            if( start > read_string.length()) start = read_string.find(startstr2);
            size_t end   = read_string.find("ms");
            size_t count = end - start -6;
            if (count > 0) {
              std::string str_time = read_string.substr(start + 5 , count);
              network_info.ping_response = std::stof(str_time);
            }
            else return false;
            


           
            int ret_val = std::remove(logfilename);
            

            return true;

          }

          catch(const std::exception& e)
          {
            return false;
          }
            

      }

      void close() {
        wifi_scan_close(wifi);
      }

				  
      


  private:
    struct wifi_scan *wifi=NULL;    //this stores all the library information
	  struct station_info station;    //this is where we are going to keep information about AP (Access Point) we are connected to
	  char mac[BSSID_STRING_LENGTH];  //a placeholder where we convert BSSID to printable hardware mac address
	  int status=0;
    
    
    
    const char *bssid_to_string(const uint8_t bssid[BSSID_LENGTH], char bssid_string[BSSID_STRING_LENGTH]) {
                  snprintf(bssid_string, BSSID_STRING_LENGTH, "%02x:%02x:%02x:%02x:%02x:%02x",
                            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
	              return bssid_string;
    }

       
};



class CPU_Info {

public:
  
  float cpu_temp;
  float cpu_perf;

  bool get_cpu_temperature() {
    bool ret_val = true;
    std::string str_temp = "";
    try
    {
       std::ifstream Read_cpu_temperature("/sys/class/thermal/thermal_zone0/temp");
       getline(Read_cpu_temperature,str_temp);
       cpu_temp = std::stof(str_temp) / 1000.0;
          
    }
    catch(const std::exception& e)
    {
      ret_val = false;
    }
      
    return ret_val;
  }


  bool  get_cpu_usage() {
          bool ret_val = true;
          
          try
            {
              size_t idle_time, total_time; 
              get_cpu_times(idle_time, total_time); 

              const float idle_time_delta = idle_time - previous_idle_time;
              const float total_time_delta = total_time - previous_total_time;
              const float utilization = 100.0 * (1.0 - idle_time_delta / total_time_delta);
              cpu_perf = utilization; 
              previous_idle_time = idle_time;
              previous_total_time = total_time;
              
            } 

          catch(const std::exception& e)
            {
              ret_val = false;          
            }

        

        return ret_val;
        }


private:


  size_t previous_idle_time=0, previous_total_time=0;

  std::vector<size_t> get_cpu_times() {
    std::ifstream proc_stat("/proc/stat");
    proc_stat.ignore(5, ' '); // Skip the 'cpu' prefix.
    std::vector<size_t> times;
    for (size_t time; proc_stat >> time; times.push_back(time));
    return times;
}

  bool get_cpu_times(size_t &idle_time, size_t &total_time) {
      const std::vector<size_t> cpu_times = get_cpu_times();
       if (cpu_times.size() < 4)
            return false;
      idle_time = cpu_times[3];
      total_time = std::accumulate(cpu_times.begin(), cpu_times.end(), 0);
      return true;
}



};





int main(int argc, char **argv)
{
 
  
  xbot_msgs::SensorInfo my_cpu_temp_info;
  xbot_msgs::SensorInfo my_cpu_perf_info;
  xbot_msgs::SensorInfo my_wifi_rssi_info;
  xbot_msgs::SensorInfo my_pingtime_info;

  // Maps a topic to a subscriber.
  std::map<std::string, ros::Subscriber> active_subscribers;
  std::map<std::string, xbot_msgs::SensorInfo> found_sensors;
  
  my_cpu_temp_info.has_critical_high = false;
  my_cpu_temp_info.has_critical_low = false;
  my_cpu_temp_info.has_min_max = true;
  my_cpu_temp_info.min_value = 0.0;
  my_cpu_temp_info.max_value = 200.0;
  my_cpu_temp_info.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
  my_cpu_temp_info.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
  my_cpu_temp_info.unit = "deg. C";
  my_cpu_temp_info.sensor_id = "om_cpu_temperature";
  my_cpu_temp_info.sensor_name = "CPU Temperature";


  my_cpu_perf_info.has_critical_high = false;
  my_cpu_perf_info.has_critical_low = false;
  my_cpu_perf_info.has_min_max = true;
  my_cpu_perf_info.min_value = 0.0;
  my_cpu_perf_info.max_value = 100.0;
  my_cpu_perf_info.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
  my_cpu_perf_info.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT;
  my_cpu_perf_info.unit = "%";
  my_cpu_perf_info.sensor_id = "om_cpu_performance";
  my_cpu_perf_info.sensor_name = "CPU Performance";

  my_wifi_rssi_info.has_critical_high = false;
  my_wifi_rssi_info.has_critical_low = false;
  my_wifi_rssi_info.has_min_max = true;
  my_wifi_rssi_info.min_value = -100.0;
  my_wifi_rssi_info.max_value = 100.0;
  my_wifi_rssi_info.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
  my_wifi_rssi_info.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
  my_wifi_rssi_info.unit = "db";
  my_wifi_rssi_info.sensor_id = "om_wifi_quality";
  my_wifi_rssi_info.sensor_name = "RSSI";

  my_pingtime_info.has_critical_high = false;
  my_pingtime_info.has_critical_low = false;
  my_pingtime_info.has_min_max = true;
  my_pingtime_info.min_value = 0.0;
  my_pingtime_info.max_value = 3000.0;
  my_pingtime_info.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
  my_pingtime_info.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
  my_pingtime_info.unit = "ms";
  my_pingtime_info.sensor_id = "om_ping";
  my_pingtime_info.sensor_name = "Ping time";


  bool fan_on = false;

#ifndef __x86_64__
  // init Raspi PIO 6 as Output for Fancontrol
  int ret_sys = system("raspi-gpio set 6 op");
#endif  

  Wifi_Quality My_WifiQuality;
  CPU_Info My_cpu_info;
  
  
  if (My_WifiQuality.wifi_channel == NULL) ROS_ERROR("Environment Variable is not set. Set: export OM_WIFI= <channel>, e.g. export OM_WIFI=wlo1");
    else 
      if (!My_WifiQuality.is_connected()) ROS_ERROR("WiFi Channel: %s not known, check with command iwconfig",My_WifiQuality.wifi_channel);
        else ROS_INFO("WiFi Channel: %s connected",My_WifiQuality.wifi_channel);


  if (My_WifiQuality.target_ping == NULL) ROS_ERROR("Environment Variable is not set. Set: export OM_TARGET_PING= <address>, e.g. export OM_TARGET_PING=192.168.1.1");


  ros::init(argc, argv, "mower_inf");
 
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<mower_addons::raspi_inf>("mower_addons/machine_info", 100);

  ros::Publisher sensor_cpu_temp_info_publisher = n.advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + my_cpu_temp_info.sensor_id + "/info", 1, true);
  ros::Publisher sensor_cpu_temp_data_publisher = n.advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + my_cpu_temp_info.sensor_id + "/data", 1, false);
  sensor_cpu_temp_info_publisher.publish(my_cpu_temp_info);


  ros::Publisher sensor_cpu_perf_info_publisher = n.advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + my_cpu_perf_info.sensor_id + "/info", 1, true);
  ros::Publisher sensor_cpu_perf_data_publisher = n.advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + my_cpu_perf_info.sensor_id + "/data", 1, false);
  sensor_cpu_perf_info_publisher.publish(my_cpu_perf_info);

  ros::Publisher sensor_pingtime_info_publisher = n.advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + my_pingtime_info.sensor_id + "/info", 1, true);
  ros::Publisher sensor_pingtime_data_publisher = n.advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + my_pingtime_info.sensor_id + "/data", 1, false);
  sensor_pingtime_info_publisher.publish(my_pingtime_info);

  ros::Publisher sensor_rssi_info_publisher = n.advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + my_wifi_rssi_info.sensor_id + "/info", 1, true);
  ros::Publisher sensor_rssi_data_publisher = n.advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + my_wifi_rssi_info.sensor_id + "/data", 1, false);
  sensor_rssi_info_publisher.publish(my_wifi_rssi_info);



  xbot_msgs::SensorDataDouble data;

  

  ros::Rate loop_rate(1);



  

  
  
  while (ros::ok())
  {
  
    

    mower_addons::raspi_inf msg;
    
    msg.access_point      = My_WifiQuality.network_info.access_point;
    msg.CPU_performance   = My_cpu_info.cpu_perf;
    msg.CPU_temperature   = My_cpu_info.cpu_temp;
    msg.mac               = My_WifiQuality.network_info.wifi_mac;
    msg.pingtime          = My_WifiQuality.network_info.ping_response;
    msg.rx_packets        = My_WifiQuality.network_info.rx_packets;
    msg.signal            = My_WifiQuality.network_info.signal;
    msg.signal_avg        = My_WifiQuality.network_info.signal_avg;
    msg.tx_packets        = My_WifiQuality.network_info.tx_packets;

    pub.publish(msg);

    if ( My_WifiQuality.wifi_channel != NULL && My_WifiQuality.is_connected() ) {
      int status = My_WifiQuality.get_station_info();
      if(status==0)
			  ROS_ERROR("No associated station\n");
		  else if(status==-1)
			  ROS_ERROR("Unable to get station information\n");
		    else
			    ROS_INFO("MAC: %s, AP: %s, Signal: %d dBm, Signal-avg: %d dBm, rx %u packets, tx %u packets\n",
				            My_WifiQuality.network_info.wifi_mac.c_str() , My_WifiQuality.network_info.access_point.c_str() , My_WifiQuality.network_info.signal,
                    My_WifiQuality.network_info.signal_avg , My_WifiQuality.network_info.rx_packets, My_WifiQuality.network_info.tx_packets);
    }                

    if (My_cpu_info.get_cpu_temperature()) ROS_INFO("CPU Temp: %.1f Grad Celsius",My_cpu_info.cpu_temp);

    if (My_cpu_info.get_cpu_usage()) ROS_INFO("CPU usage: %.1f %%",My_cpu_info.cpu_perf);

    if (My_WifiQuality.get_station_ping()) ROS_INFO("ping response: %.1f ms",My_WifiQuality.network_info.ping_response);



#ifndef __x86_64__
    // control CPU-Fan
    if (!fan_on && My_cpu_info.cpu_temp > CPU_MAX_TEMP) {
         int ret_sys = system("raspi-gpio set 6 dh");
         ROS_INFO("CPU fan ON\n");
         fan_on = true;
        }  
    else
      if (fan_on && My_cpu_info.cpu_temp < CPU_MIN_TEMP) {
          int ret_sys = system("raspi-gpio set 6 dl");
          ROS_INFO("CPU fan OFF\n");
          fan_on = false;
      }
#endif



    data.stamp = ros::Time::now();
    data.data = My_cpu_info.cpu_temp;
    sensor_cpu_temp_data_publisher.publish(data);
    
    data.data = My_cpu_info.cpu_perf;
    sensor_cpu_perf_data_publisher.publish(data);

    data.data = My_WifiQuality.network_info.ping_response;
    sensor_pingtime_data_publisher.publish(data);

    data.data = My_WifiQuality.network_info.signal;
    sensor_rssi_data_publisher.publish(data);



    ros::spinOnce();

    loop_rate.sleep();
    
  }

  //wrap up
  My_WifiQuality.close();



  return 0;
}
