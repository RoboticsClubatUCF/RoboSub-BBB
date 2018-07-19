#include "sub_thruster_library/t200_thruster.h"
//#include "sub_thruster_library/seabotix_thruster.h"
#include "sub_thruster_library/generic_thruster.h"

#include <json/json.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/SelfTest.h> //TODO: implement self tests

#include <iostream>
#include <fstream>

//TODO: Determine module name at runtime (needs some capemgr fuckery)
#define SUB_SECTION_NAME "COMPUTE"
#define CONFIG_FILE_PATH "config.json"

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;
    self_test::TestRunner self_test_;

    sub_trajectory::ThrusterCmd savedMsg;

    std::map<int, GenericThruster> thrusterMap;

    ros::ServiceServer initServer;

public:
    ThrusterManager() : spinner(1), self_test_()
    {
        for(int i = 0; i < 8; i++) {
            savedMsg.cmd.push_back(0.0);
        }

        command_subscriber = nh_.subscribe("/thrusters/cmd_vel", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1000);

        self_test_.add("Test connections", this, &ThrusterManager::testThrusterConnections);

        initServer = nh_.advertiseService("initThrusters", &ThrusterManager::initService, this);

    	//thrusterMap[0] = T200Thruster(1, 0x2D);
    	//thrusterMap[1] = T200Thruster(1, 0x2E); //This should keep a reference? The copy disabling thing should keep us safe
    }

    Json::Value loadConfig(std::string filename)
    {
        ifstream configFile(filename);
        if(!ifstream.is_open()){
            ROS_ERROR("%s thruster controller couldn't open config file", SUB_SECTION_NAME);
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Config file didn't load");
            return;
        }

        Json::Reader reader;
        Json::Value obj;
        reader.parse(configFile, obj);
        return obj;
    }
    void initService(std_msgs::Empty& req, std_msgs::Empty& resp)
    {
        init();
    }

    void init()
    {
        thrusterMap.clear();
        Json::Value& thrustersJson = loadConfig(CONFIG_FILE_PATH)[SUB_SECTION_NAME];
        for(int i = 0; i < thrustersJson.size(); i++) {
            int thrusterID = thrustersJson[i]["ID"].asInt();
            int thrusterType = thrustersJson[i]["Type"].asInt(); //TODO: support for multiple thruster types
            int thrusterAddress = thrustersJson[i]["Address"].asInt();
            try{
                thrusterMap[thrusterID] = T200Thruster(1, thrusterAddress);
            }
            catch(I2CException e){
                //If we get here there's a bus problem
                //Publish an error message for the diagnostic system to do something about
                diagnostic_msgs::DiagnosticStatus status;
                status.name = "Thrusters";
                status.hardware_id = "Thrusters";
                status.level = status.ERROR;
                diagnostics_output.publish(status);
                ros::spinOnce();
            }
            else {

            }
        }
    }

    void spin()
    {
        ros::Rate rate(10);
        while(ros::ok()) {
            //Publish diagnostic data here
            diagnostic_msgs::DiagnosticStatus status;
            status.name = "Thrusters";
            status.hardware_id = "Thrusters"; //TODO: Different hardware ID/section based on sub section name?

			for(auto& thruster : thrusterMap)
			{
                try {
                    iter.second.updateStatus();
                    iter.second.setVelocity(savedMsg.cmd.at(iter.first));
                } catch(I2CException e) {
                    //Publish an error message for the diagnostic system to do something about
                    status.level = status.ERROR;
                }

				if (thrusterOk(iter.second) && status.level != status.ERROR)
					status.level = status.OK;
				else
					status.level = status.ERROR;

				PushDiagData(status, iter.second, std::to_string(iter.first));
            }

            diagnostics_output.publish(status);
            ros::spinOnce();
            self_test_.checkTest();
            rate.sleep();
        }
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

    //Self test function
    void testThrusterConnections(diagnostic_updater::DiagnosticStatusWrapper& status)
    {
        self_test_.setID("thrusterController");
        std::stringstream failedThrusters;
        Json::Value& thrustersJson = loadConfig(CONFIG_FILE_PATH)[SUB_SECTION_NAME];
        for(int i = 0; i < thrustersJson.size(); i++)
		{
            int thrusterID = thrustersJson[i]["ID"].asInt();
            int thrusterType = thrustersJson[i]["Type"].asInt(); //TODO: support for multiple thruster types
            int thrusterAddress = thrustersJson[i]["Address"].asInt();

            try{
                T200Thruster(1, thrusterAddress).updateStatus();
            }
            catch(I2CException e){
                failedThrusters << thrusterAddress << " ";
            }
        }

        if(failedThrusters.str().length() > 0){
            status.add("Failures", failedThrusters.str());
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Thruster couldn't connect");
        } else {
            status.summary(diagnostic_msgs::DiagnosticStatus::OK, "All thrusters OK");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thruster_driver");
    ThrusterManager tc;
    tc.init();
    tc.spin();
    return 0;
}

inline const std::string BoolToString(const bool b)
{
  return b ? "true" : "false";
}
