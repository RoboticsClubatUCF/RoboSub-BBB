#include "blue_robotics_t200/t200_thruster.h"
#include "seabotix_thruster/seabotix_thruster.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/SelfTest.h>

#define THRUSTER_UNDERVOLT 11.0
#define THRUSTER_OVERVOLT 20.0
#define THRUSTER_OVERCURRENT 25.0
#define THRUSTER_OVERTEMP 40.0

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;

    ros::AsyncSpinner spinner;

    T200Thruster thrusterr;
    T200Thruster thrusterl;
    
    float tFrontUp = 0.0f, tRearUp = 0.0f;
    float tLeftForward = 0.0f, tRightForward = 0.0f;
    float tTopStrafe = 0.0f, tBottomStrafe = 0.0f;

public:
    ThrusterManager() : thrusterl(1, 0x2D), thrusterr(1, 0x2E), spinner(1)
    {
        command_subscriber = nh_.subscribe("/thrusters/cmd_vel", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1000);
    }

    void init()
    {
        if(spinner.canStart())
            spinner.start();
        else
            return;

        ros::Rate rate(4);
        while(ros::ok()) {
            //Publish diagnostic data here
            diagnostic_msgs::DiagnosticStatus status;
            status.name = "Thrusters";
            status.hardware_id = "Thrusters";

            thrusterr.updateStatus();
            thrusterl.updateStatus();
            
            if (thrusterOk(thrusterr) && thrusterOk(thrusterl))
                status.level = status.OK;
            else
                status.level = status.ERROR;

            PushDiagData(status, thrusterr, "Thruster R");
            PushDiagData(status, thrusterl, "Thruster L");
            
            thrusterr.setVelocityRatio(tLeftForward);
            thrusterl.setVelocityRatio(tRightForward);

            rate.sleep();
        }
        spinner.stop();
    }
    
    bool thrusterOk (T200Thruster & thruster)
    {
        return thruster.isAlive() && THRUSTER_OVERVOLT > thruster.getVoltage() > THRUSTER_UNDERVOLT && THRUSTER_OVERCURRENT > thruster.getCurrent() && THRUSTER_OVERTEMP > thruster.getTemperature();
    }

    bool thrusterOk (SeabotixThruster & thruster)
    {
        return thruster.isAlive() && THRUSTER_OVERCURRENT > thruster.getCurrent() && THRUSTER_OVERTEMP > thruster.getTemperature();
    }
    
    void PushDiagData(diagnostic_msgs::DiagnosticStatus & statusmsg, SeabotixThruster & thruster, std::string thrusterName)
    {
        diagnostic_msgs::KeyValue thrusterValue;
        
        thrusterValue.key = "Thruster Type";
        thrusterValue.value = "Seabotix";
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Alive";
        thrusterValue.value = BoolToString(thruster.isAlive());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Current";
        thrusterValue.value = std::to_string(thruster.getCurrent());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Temperature";
        thrusterValue.value = std::to_string(thruster.getTemperature());
        statusmsg.values.push_back(thrusterValue);
    }
    
    void PushDiagData(diagnostic_msgs::DiagnosticStatus & statusmsg, T200Thruster & thruster, std::string thrusterName)
    {
        diagnostic_msgs::KeyValue thrusterValue;

        thrusterValue.key = "Thruster Type";
        thrusterValue.value = "Blue Robotics T200";
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

    void thrusterCb(const geometry_msgs::Twist &msg)
    {
	//copy data out of the message so we can modify as necessary
        float angularX, angularY, angularZ, linearX, linearY, linearZ;

        angularX = msg.angular.x;
        angularY = msg.angular.y;
        angularZ = msg.angular.z;

        linearX = msg.linear.x;
        linearY = msg.linear.y;
        linearZ = msg.linear.z;

        //normalize linear/angular pairs that depend on the same thruster pairs
        /*
          We need this because trying to apply a tranlation force of +/-1 and a torque of +/-1 isnt possible
          with our thruster setup, so we normalize in order to approximate whatever mix the controller wants
        */

        if(magnitude(linearZ, angularY) > 1.0)
        {
            linearZ = linearZ / magnitude(linearZ, angularY);
            angularY = angularY /  magnitude(linearZ, angularY);
        }

        if(magnitude(linearX, angularZ) > 1.0)
        {
            linearX = linearX / magnitude(linearX, angularZ);
            angularZ = angularZ /  magnitude(linearX, angularZ);
        }

        if(magnitude(linearY, angularX) > 1.0)
        {
            linearY = linearY / magnitude(linearY, angularX);
            angularX = angularX /  magnitude(linearY, angularX);
        }

        //mix the twist message, limits for extra safety
        tFrontUp = std::max(-1.0f, std::min(1.0f, linearZ + angularY));
        tRearUp = std::max(-1.0f, std::min(1.0f, linearZ  - angularY));

        tLeftForward = std::max(-1.0f, std::min(1.0f, linearX - angularZ));
        tRightForward = std::max(-1.0f, std::min(1.0f, linearX + angularZ));

        tTopStrafe = std::max(-1.0f, std::min(1.0f, linearY - angularX));
        tBottomStrafe = std::max(-1.0f, std::min(1.0f, linearY + angularX));
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
