#ifndef Seabotix_Thruster_H
#include "i2c_interface.h"

namespace SeabotixThrusterDirections
{
    enum SeabotixThrusterDirection
    {
        Forward,
        Reverse
    };
}
typedef SeabotixThrusterDirections::SeabotixThrusterDirection SeabotixThrusterDirection;

namespace SeabotixThrusterStatusIndices
{
    enum SeabotixThrusterStatusIndex
    {
        Status,
        Fault,
        Temperature,
        Current,
        Checksum
    };
} 
typedef SeabotixThrusterStatusIndices::SeabotixThrusterStatusIndex SeabotixThrusterStatusIndex;

////////////////////////////////////////////////////////////////////////////////
/// \brief Class providing functionality for using the Blue Robotics Blue ESC Seabotix
/// 
/// This class provides for control and monitoring of Blue Robotics Seabotix Thrusters
/// using the Blue ESC in I2C mode.  It supports velocity control as a ratio of 0.0
/// to 1.0 in forward and reverse directions.
///
////////////////////////////////////////////////////////////////////////////////
class SeabotixThruster
{
public:
////////////////////////////////////////////////////////////////////////////////
/// \brief Constructs a new SeabotixThruster interface.
/// 
/// \param bus_number   The I2C bus to use.
/// \param address      The 7-bit slave address of the thruster.
////////////////////////////////////////////////////////////////////////////////
SeabotixThruster(int bus_number, unsigned char address);

////////////////////////////////////////////////////////////////////////////////
// \brief Default SeabotixThruster destructor.
//
////////////////////////////////////////////////////////////////////////////////
~SeabotixThruster();

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Sets the thruster power as a ratio of the maximum supported velocity by the 
/// thruster and according to the direction supplied.
///
/// \param velocity_scale    The ratio between 0.0 and 1.0 for thruster power.
/// \param direction         The direction of the thruster.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_scale, SeabotixThrusterDirection direction);

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

////////////////////////////////////////////////////////////////////////////////
/// \brief Forces the SeabotixThruster to update its most recent status data.
///
/// Initiates communication to the thruster and retrieves all status bytes
/// supported by the Blue ESC for use in status exposure functions.
////////////////////////////////////////////////////////////////////////////////
void updateStatus();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the temperature of the ESC in degrees Celsius.
///
////////////////////////////////////////////////////////////////////////////////
double getTemperature();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the current supplied to the thruster.
///
////////////////////////////////////////////////////////////////////////////////
double getCurrent();

////////////////////////////////////////////////////////////////////////////////
/// \brief Checks that the ESC is communicating as expected.
///
////////////////////////////////////////////////////////////////////////////////
bool isAlive();

private:

static const int STATUS_DATA_BYTES = 6;
static const int VELOCITY_SCALE = 0x66;
static const int ZERO_VELOCITY_VALUE = 0x80;
static const unsigned char THROTTLE_REGISTER_START = 0x52;
static const unsigned char DATA_REGISTER_START = 0x53;
static const unsigned char GOOD_IDENTIFIER = 0xAB;

ByteBuffer status_data_;
I2C_Interface i2c_interface_;

void setVelocity(int velocity_command);
int getRawTemperatureMeasurement();
int getRawCurrentMeasurement();
unsigned char getIdentifier();
};

#endif
