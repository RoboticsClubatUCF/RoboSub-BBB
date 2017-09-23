#include "sub_thruster_library/t200_thruster.h"
//#include "sub_thruster_library/seabotix_thruster.h"
#include "sub_thruster_library/generic_thruster.h"

#include <json/json.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/SelfTest.h>

#include <iostream>
#include <fstream>

//TODO: Determine module name at runtime (needs some capemgr fuckery)
#define SUB_SECTION_NAME "COMPUTE"

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;

    sub_trajectory::ThrusterCmd savedMsg;
    ros::AsyncSpinner spinner;
    
    std::map<int, GenericThruster> thrusterMap;

public:
    ThrusterManager() : spinner(1)
    {
        for(int i = 0; i < 8; i++) {
            savedMsg.cmd.push_back(0.0);
        }

        command_subscriber = nh_.subscribe("/thrusters/cmd_vel", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1000);
    	
    	//thrusterMap[0] = T200Thruster(1, 0x2D);
    	//thrusterMap[1] = T200Thruster(1, 0x2E); //This should keep a reference? The copy disabling thing should keep us safe
    }

    void init()
    {
        if(spinner.canStart())
            spinner.start();
        else
            return;   
        
        ifstream configFile("config.json");
        if(!ifstream.is_open()){
            ROS_ERROR("%s thruster controller couldn't open config file", SUB_SECTION_NAME);
            return;
        }
        
        Json::Reader reader;
        Json::Value obj;
        reader.parse(configFile, obj);

        Json::Value& thrustersJson = obj[SUB_SECTION_NAME];
        for(int i = 0; i < thrustersJson.size(); i++) {
            int thrusterID = thrustersJson[i]["ID"].asInt();
            int thrusterType = thrustersJson[i]["Type"].asInt();
            int thrusterAddress = thrustersJson[i]["Address"].asInt();
            thrusterMap[thrusterID] = T200Thruster(1, thrusterAddress);
        }
        //TODO: Iterate through the thruster param and build thruster objects.
    	//No fucking idea how you build things from JSON in C++ but ROS does it somehow.

        ros::Rate rate(4);
        while(ros::ok()) {
            //Publish diagnostic data here
            diagnostic_msgs::DiagnosticStatus status;
            status.name = "Thrusters";
            status.hardware_id = "Thrusters";
            
			for(auto& thruster : thrusterMap)
			{
				iter.second.updateStatus();
				
				if (thrusterOk(iter.second) && status.level != status.ERROR)
					status.level = status.OK;
				else
					status.level = status.ERROR;
				
				PushDiagData(status, iter.second, std::to_string(iter.first));
				
				iter.second.setVelocity(savedMsg.cmd.at(iter.first));
            }
            
            diagnostics_output.publish(status);
            rate.sleep();
        }
        spinner.stop();
    }
    
    bool thrusterOk (GenericThruster & thruster)
    {
        return thruster.isAlive() && thruster.inLimits();
    }
    
    void PushDiagData(diagnostic_msgs::DiagnosticStatus & statusmsg, GenericThruster & thruster, std::string thrusterName)
    {
        diagnostic_msgs::KeyValue thrusterValue;

        thrusterValue.key = "Thruster Type";
        thrusterValue.value = thruster.getType();
        statusmsg.values.push_back(thrusterValue);
        
        thrusterValue.key = thrusterName + " Alive";
        thrusterValue.value = BoolToString(thruster.isAlive());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Voltage";
        thrusterValue.value = std::to_string(thruster.getVoltage());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Current";
        thrusterValue.value = std::to_string(thruster.getCurrent());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Temperature";
        thrusterValue.value = std::to_string(thruster.getTemperature());
        statusmsg.values.push_back(thrusterValue);
    }

    void thrusterCb(const sub_trajectory::ThrusterCmd &msg)
    {
		savedMsg = msg;
    }
    
    float magnitude(float x, float y) //return the magnitude of a 2d vector
    {
        return sqrt(x*x + y*y);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thruster_driver");
    ThrusterManager tc;
    tc.init();

    return 0;
}

inline const std::string BoolToString(const bool b)
{
  return b ? "true" : "false";
}
