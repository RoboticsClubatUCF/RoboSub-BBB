#include "sub_thruster_library/t200_thruster.h"
#include "sub_thruster_library/seabotix_thruster.h"
#include "sub_thruster_library/generic_thruster.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/SelfTest.h>

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;

    ros::AsyncSpinner spinner;
    
    std::map<int, GenericThruster> thrusterMap;

public:
    ThrusterManager() : spinner(1)
    {
        command_subscriber = nh_.subscribe("/thrusters/cmd_vel", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1000);
    	
    	thrusterMap[0] = T200Thruster(1, 0x2D);
    	thrusterMap[1] = T200Thruster(1, 0x2E); //This should keep a reference? The copy disabling thing should keep us safe
    }

    void init()
    {
        if(spinner.canStart())
            spinner.start();
        else
            return;
            
    	//TODO: Iterate through the thruster param and build thruster objects.
    	//No fucking idea how you build things from JSON in C++ but ROS does it somehow.

        ros::Rate rate(4);
        while(ros::ok()) {
            //Publish diagnostic data here
            diagnostic_msgs::DiagnosticStatus status;
            status.name = "Thrusters";
            status.hardware_id = "Thrusters";
            
			for(std::map<int, GenericThruster>::iterator iter = thrusterMap.begin(); iter != thrusterMap.end(); iter++)
			{
				iter->second.updateStatus();
				
				if (thrusterOk(iter->second) && status.level != status.ERROR)
					status.level = status.OK;
				else
					status.level = status.ERROR;
				
				PushDiagData(status, iter->second, std::to_string(iter->first));
				
				iter->second.setVelocity(msg.at(iter->first));
			}
			
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
