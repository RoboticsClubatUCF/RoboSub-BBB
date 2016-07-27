#ifndef T200_Thruster_H
#include "i2c_interface.h"
#include "BlackPWM/BlackPWM.h"

namespace ThrusterPWMDirections
{
    enum ThrusterPWMDirection
    {
        Forward,
        Reverse
    };
}
typedef ThrusterPWMDirections::ThrusterPWMDirection ThrusterPWMDirection;

////////////////////////////////////////////////////////////////////////////////
/// \brief Class providing functionality for using the Blue Robotics Blue ESC T200
/// 
/// This class provides for control and monitoring of Blue Robotics T200 Thrusters
/// using the Blue ESC in I2C mode.  It supports velocity control as a ratio of 0.0
/// to 1.0 in forward and reverse directions.
///
////////////////////////////////////////////////////////////////////////////////
class ThrusterPWM
{
public:
////////////////////////////////////////////////////////////////////////////////
/// \brief Constructs a new ThrusterPWM interface.
/// 
/// \param bus_number   The I2C bus to use.
/// \param address      The 7-bit slave address of the thruster.
////////////////////////////////////////////////////////////////////////////////
ThrusterPWM(BlackLib::pwmName);

////////////////////////////////////////////////////////////////////////////////
// \brief Default ThrusterPWM destructor.
//
////////////////////////////////////////////////////////////////////////////////
~ThrusterPWM();

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Sets the thruster power as a ratio of the maximum supported velocity by the 
/// thruster and according to the direction supplied.
///
/// \param velocity_scale    The ratio between 0.0 and 1.0 for thruster power.
/// \param direction         The direction of the thruster.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_scale, ThrusterPWMDirection direction);

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Overload to set thruster direction and power as a ratio of maximum supported velocity
///
/// \param velocity_ratio    The ratio between -1.0 and 1.0 for thruster power.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_ratio);

////////////////////////////////////////////////////////////////////////////////
/// \brief linearizes thruster output and almost eliminates deadzone
///
/// Function that takes desired thrust output as a fraction from 1.0 to -1.0 and
/// applies a linear function based on data from the manufacturer so that deadband is 
/// reduced to a range from 0.01 to -0.01 and maximum output is the same in both 
/// directions
///
/// \param velocity_desired desired thruster output
////////////////////////////////////////////////////////////////////////////////
double linearizeOutput(double velocity_desired);


private:

BlackLib::BlackPWM pwm_interface_;

void setVelocity(double target_ratio);
};

#endif
