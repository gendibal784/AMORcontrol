#ifndef AMOR_H
#define AMOR_H

/*! @file
 *  @author Floran Stuyt (f.stuyt@exactdynamics.nl)
 *  @author Rick ten Brink (r.tenbrink@exactdynamics.nl)
 *  @date   February, 2012
 *  @brief  Contains all AMOR library function declarations.
 *
 *  Include this file in your project, and make sure that you tell
 *  the linker to link to amor_api.lib. Furthermore, ensure that
 *  the amor_api.dll file is located in the working directory of
 *  your application.

 * @defgroup AMOR The AMOR API library
*/
#include "amor_export.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * @addtogroup AMOR
 * @{
*/

// TYPE DEFINITIONS

/// Represents a handle to the AMOR API
typedef void*   AMOR_HANDLE;
/// Represents the result from calling an AMOR API library function
typedef int     AMOR_RESULT;
/// In the AMOR API library, reals are represented as doubles
typedef double  real;

/// Represents successfull excution of a function
#define         AMOR_SUCCESS                  0
/// Represents unsuccessfull excution of a function. See amor_errno and amor_error for details
#define         AMOR_FAILED                   -1
/// Returned from amor_connect if the function call fails. See amor_errno and amor_error for details
#define         AMOR_INVALID_HANDLE           0

/// There is currently no warning
#define         AMOR_NO_WARNING               0
/// The joint temperature is too high/low
#define         AMOR_WARN_TEMPERATURE         1
/// The joint has reached the lower end-switch
#define         AMOR_WARN_LOWER_END_SWITCH    2
/// The joint has reached the upper end-switch
#define         AMOR_WARN_UPPER_END_SWITCH    3
/// The setpoint buffer is empty
#define         AMOR_WARN_BUFFER_EMPTY        5

/// AMOR amount of joints
#define         AMOR_NUM_JOINTS               7

/// Returned from amor_error when no error is
#define         AMOR_ERROR_NONE               0
/// An unknown error has occured
#define         AMOR_ERROR_UNKNOWN            1
/// Returned when the connection with the AMOR failed
#define         AMOR_ERROR_CONNECTION_FAILURE 2
/// Returned when an invalid argument has been supplied
#define         AMOR_ERROR_INVALID_ARGUMENT   3
/// Returned when there are communication problems
#define         AMOR_ERROR_COMMUNICATION      4
/// Returned when the CAN driver fails
#define         AMOR_ERROR_CAN_DRIVER         5

/// @brief Representation of joint limit information
typedef struct tagAMOR_JOINT_INFO {
    /// maximum current that may be applied to the joint motor in [mA] units
    real maxCurrent;
    /// maximum voltage that may be applied to the joint motor in [mV] units
    real maxVoltage;
    /// maximum velocity of the joint in [rad/s] units
    real maxVelocity;
    /// maximum acceleration of the joint in [rad/s^2] units
    real maxAcceleration;
    /// lower joint angle limit in [rad] units
    real lowerJointLimit;
    /// upper joint angle limit in [rad] units
    real upperJointLimit;
} AMOR_JOINT_INFO;

/// @brief Representation of joint firmware version
typedef struct tagAMOR_JOINT_VERSION {
    /// software major version
    unsigned char major;
    /// software minor version
    unsigned char minor;
    /// software revision version
    unsigned short revision;
    /// module parameters version number
    unsigned char parameters_version;
} AMOR_JOINT_VERSION;

/// @brief Representation of a 7 dimensional real vector
typedef real AMOR_VECTOR7[AMOR_NUM_JOINTS];

/// @brief Struct representing a 4x4 matrix
typedef struct tagAMOR_MATRIX44 {
    union {
        struct {
            real        _11, _12, _13, _14;
            real        _21, _22, _23, _24;
            real        _31, _32, _33, _34;
            real        _41, _42, _43, _44;
        };
        real m[4][4];
    };
} AMOR_MATRIX44;

/// @brief Current status of the movement
enum amor_movement_status{
    /// AMOR is moving
    AMOR_MOVEMENT_STATUS_MOVING,
    /// Movement is pending
    AMOR_MOVEMENT_STATUS_PENDING,
    /// AMOR reached the requested position
    AMOR_MOVEMENT_STATUS_FINISHED
};

/// @brief Request AMOR API library version information.
///
/// This function can be used to request version information of the
/// AMOR library. The version number is constructed by
/// a triplet consisting of a major, minor, and build number.
///
/// @param[out] major major part of version number
/// @param[out] minor minor part of version number
/// @param[out] build build part of version number
AMOR_EXPORT void amor_get_library_version(int* major, int* minor, int* build);

/// @brief Open/initiate a connection to AMOR.
///
/// Before it is possible to control AMOR with this API, a
/// connection needs to be initiated. This function will open and
/// initiate the connection to AMOR. The handle that is
/// returned should be stored and supplied to all functions of
/// this API. It is possible to open multiple connections to multiple AMOR connected
/// to the PC, however it is not possible to open multiple
/// connections to a single AMOR.
///
/// @note Make sure AMOR is powered before calling this function.
///
/// @param[in] libraryName a null terminated string referring to the canlib dll
/// driver that should be used to connect to the CAN interface.
/// @param[in] can_port    CAN device net number to which AMOR is connected
/// Please refer to the manual of the CAN interface device for information
/// about installing and using the CAN device.
/// @return If successful, a valid AMOR_HANDLE is returned. In case of an error
/// AMOR_INVALID_HANDLE is returned. Use amor_errno() for
/// requesting the corresponding error code, and amor_error() for
/// a textual representation of the error.
AMOR_EXPORT AMOR_HANDLE amor_connect(char* libraryName, int can_port);

/// @brief Specify the currents that should be applied to each joint motor
///
/// If applicable, sets all AMOR joints in NORMAL_CURRENT_MODE and applies the
/// given currents to the joints.
///
/// @param[in] handle   handle to the AMOR robot
/// @param[in] currents currents that should be applied to the motor joints in [mA] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_currents(AMOR_HANDLE handle, AMOR_VECTOR7 currents);

/// @brief Retrieve the requested motor currents
///
/// Basically, this function returns the currents requested using the amor_set_currents function.
///
/// @param[in] handle   handle to the AMOR robot
/// @param[out] currents requested motor currents in [mA] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_req_currents(AMOR_HANDLE handle, AMOR_VECTOR7 *currents);

/// @brief Retrieve the currents that are currently measured at the joint motors
///
/// AMOR is requested to send the currents that have been measured at
/// the joint motors.
///
/// @param[in] handle    handle to the AMOR robot
/// @param[out] currents motor currents measured by the robot  [mA] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_actual_currents(AMOR_HANDLE handle, AMOR_VECTOR7 *currents);

/// @brief Specify the voltages that should be applied to each joint motor
///
/// Sets all AMOR joints in NORMAL_VOLTAGE_MODE and applies the given voltages to the joints.
///
/// @param[in] handle   handle to the AMOR robot
/// @param[in] voltages voltages that should be applied to the motor joints in [mV] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_voltages(AMOR_HANDLE handle, AMOR_VECTOR7 voltages);

/// @brief Retrieve the requested motor voltages
///
/// Basically this function returns the voltages requested using the amor_set_voltages function.
///
/// @param[in] handle   handle to the AMOR robot
/// @param[out] voltages requested motor voltages in [mV] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_req_voltages(AMOR_HANDLE handle, AMOR_VECTOR7 *voltages);

/// @brief Retrieve the voltages that are currently measured at the joint motors
///
/// AMOR is requested to send the voltages that have been measured at
/// the joint motors.
///
/// @param[in] handle   handle to the AMOR robot
/// @param[out] voltages motor voltages measured by the robot in [mV] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_actual_voltages(AMOR_HANDLE handle, AMOR_VECTOR7* voltages);

/// @brief Specify the joints angles the robot should try to move to
///
/// If applicable, sets all AMOR joints in NORMAL_POSITION_MODE and generates a trajectory
/// for the given positions. The generated trajectory has a trapezoidal velocity profile
/// computed using the joint velocity and acceleration limits. The trajectory
/// is generated online and trajectory points are sent to the robot each time the robot
/// performs a PID control cycle. See the Instructions for Use document for details.
///
/// @param[in] handle    handle to the AMOR robot
/// @param[in] positions target position of the robot in [rad] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_positions(AMOR_HANDLE handle, AMOR_VECTOR7 positions);

/// @brief Request the joints angles the robot was last commanded to move to
///
/// @param[in]  handle    handle to the AMOR robot
/// @param[out] positions the last commanded joint angles
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_req_positions(AMOR_HANDLE handle, AMOR_VECTOR7* positions);

/// @brief Retrieve the actual joints angles of the robot
///
/// Retrieve the actual joint angles of the robot. Depending on the mode of operation of the robot
/// two methods may be used to retreive the actual joint angles. In MODE_IDLE, this function
/// returns the angle from the optical angle measurement system. Otherwise, it will return
/// the angle calculated from an initial optical angle encoders read-out and subsequent pulses from
/// the incremental angle encoders.
///
/// @param[in]  handle    handle to the AMOR robot
/// @param[out] positions joint angles measured by the robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_actual_positions(AMOR_HANDLE handle, AMOR_VECTOR7* positions);

/// @brief Specify the joint velocities the robot should try achieve
///
/// If applicable, sets all AMOR joints in NORMAL_POSITION_MODE and generates a trajectory
/// using the given velocites. The generated trajectory has a trapezoidal velocity profile
/// computed using the joint velocity and acceleration limits. The trajectory
/// is generated online and trajectory points are sent to the robot each time the robot
/// performs a PID control cycle. See the Instructions for Use document for details.
///
/// @param[in] handle     handle to the AMOR robot
/// @param[in] velocities target velocities of the robot joints in [rad] units
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_velocities(AMOR_HANDLE handle, AMOR_VECTOR7 velocities);

/// @brief Request the joint velocities the robot was last commanded to achieve
///
/// @param[in]  handle     handle to the AMOR robot
/// @param[out] velocities the last commanded joint velocities
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_req_velocities(AMOR_HANDLE handle, AMOR_VECTOR7* velocities);

/// @brief Retrieve the actual joints angles of the robot
///
/// Retrieve the actual joint angles of the robot. Depending on the mode of operation of the robot
/// two methods may be used to retreive the actual joint angles. In MODE_IDLE, this function
/// returns the angle from the optical angle measurement system. Otherwise, it will return
/// the angle calculated from an initial optical angle encoders and subsequent pulses from
/// the incremental angle encoders.
///
/// @param[in]  handle    handle to the AMOR robot
/// @param[out] velocities joint velocities measured by the robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_actual_velocities(AMOR_HANDLE handle, AMOR_VECTOR7* velocities);

/// @brief Command an emergency stop to the robot
///
/// This function is almost equal to amor_controlled stop.
/// The robot will decelerate to zero velocity as fast as permissible
/// by the joint acceleration limits. Note that this function
/// does not aim to make all joints stop simultaneously. Each
/// joint is stopped as quick as possible. This function differs from
/// amor_controlled_stop in that it will command all joints to
/// go to MODE_IDLE approx. 250 milliseconds after calling this function.
///
/// \warning Even while the robot may not have reached zero velocity within this
/// time interval brakes will be applied to the joints equipped with a braking
/// system. Depending on the payload joints without a brake may continue moving
/// as a result of their angular momentum.
///
/// \note This function is not a replacement for any safety related
/// equipment such as an emergency switch.
///
/// @param[in]  handle   handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_emergency_stop(AMOR_HANDLE handle);

/// @brief Command a controlled stop to the robot
///
/// The robot will decelerate to zero velocity as fast as permissible
/// by the joint acceleration limits. Note that this function
/// does not aim to make all joints stop simultaneously. Each
/// joint is stopped as quick as possible. After stopping,
/// the robot remains in NORMAL_POSITION_MODE. Thus, no brakes
/// will be applied and the robot will aim to keep the position
/// reached after stopping.
///
/// \note This function is not a replacement for any safety related
/// equipment such as an emergency switch.
///
/// @param[in]  handle   handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_controlled_stop(AMOR_HANDLE handle);

/// @brief Release the handle to the AMOR robot
///
/// Always call this function before closing your application.
///
/// @param[in]  handle   handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_release(AMOR_HANDLE handle);

/// @param[in]  handle   handle to the AMOR robot
/// @param[in]  joint    index of the joint of interest (0 = A1, 1 = A2, 2 = A2.5, 3=A3, ...)
/// @param[out] status
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_status(AMOR_HANDLE handle, int joint, int *status);

/// @brief Retrieve limit information of an AMOR joint
///
/// Every AMOR joint has a maximum velocity, maximum acceleration, maximum voltage/current
/// and angle limits (see the Instructions for Use for details). This function can be
/// used to retrieve these values for a specific joint.
///
/// @param[in]  handle     handle to the AMOR robot
/// @param[in]  joint      index of the joint of interest (0 = A1, 1 = A2, 2 = A2.5, 3=A3, ...)
/// @param[out] parameters joint limit information
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_joint_info(AMOR_HANDLE handle, int joint, AMOR_JOINT_INFO* parameters);

/// @brief Retrieve current position and velocity movement status
///
/// When using joint position function, this function tells when all corresponding setpoints are
/// sent to the AMOR. When using controlled stop, this function can be used to see if the AMOR has already
/// stopped moving.
///
/// @param[in]  handle     handle to the AMOR robot
/// @param[out] status     AMOR movement status
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_movement_status(AMOR_HANDLE handle, amor_movement_status* status);

/// @brief Specify the cartesian joint velocities the robot should try achieve
///
/// If applicable, sets all AMOR joints in NORMAL_POSITION_MODE and generates a trajectory
/// using the given velocites. The generated trajectory has a trapezoidal velocity profile
/// computed using the joint velocity and acceleration limits. The trajectory
/// is generated online and trajectory points are sent to the robot each time the robot
/// performs a PID control cycle. See the Instructions for Use document for details.
///
/// @param[in] handle     handle to the AMOR robot
/// @param[in] velocities target velocities of the robot joints in [mm] for positions and [rad] units for angles
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_cartesian_velocities(AMOR_HANDLE handle, AMOR_VECTOR7 velocities);

/// @brief Specify the cartesian position the robot should try to move to
///
/// If applicable, sets all AMOR joints in NORMAL_POSITION_MODE and generates a trajectory
/// for the given positions. The generated trajectory has a trapezoidal velocity profile
/// computed using the joint velocity and acceleration limits. The trajectory
/// is generated online and trajectory points are sent to the robot each time the robot
/// performs a PID control cycle. See the Instructions for Use document for details.
///
/// @param[in] handle    handle to the AMOR robot
/// @param[in] positions target position of the robot in [mm] for positions and [rad] units for angles
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_set_cartesian_positions(AMOR_HANDLE handle, AMOR_VECTOR7 positions);

/// @brief Retrieve the actual cartesian position of the robot
///
/// Calculates the actual cartesian position of the robot.
/// The output vector contains the X, Y, Z position in mm. and the Yaw, Pitch and Roll angles in radians.
///
/// @param[in]  handle      handle to the AMOR robot
/// @param[out] positions   actual position of the robot in [mm] for positions and [rad] units for angle
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_get_cartesian_position(AMOR_HANDLE handle, AMOR_VECTOR7 &positions);

/// @brief Open hand of robot
///
/// Opens the hand of the robot, no check on actual pose (TODO).
///
/// @param[in]  handle      handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_open_hand(AMOR_HANDLE handle);

/// @brief Close hand of robot
///
/// Closes the hand of the robot, no check on actual pose (TODO).
///
/// @param[in]  handle      handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_close_hand(AMOR_HANDLE handle);

/// @brief Stops hand of robot
///
/// Stops the hand of the robot, no check on actual pose (TODO).
///
/// @param[in]  handle      handle to the AMOR robot
///
/// @return AMOR_SUCCESS if succesfull, AMOR_FAILED otherwise
AMOR_EXPORT AMOR_RESULT amor_stop_hand(AMOR_HANDLE handle);

/// @return The error number describing the error reported from the last erroneous function call
AMOR_EXPORT unsigned int amor_errno();

/// @return A null-terminated string describing the error reported from the last erroneous function call
AMOR_EXPORT const char* amor_error();

/*@}*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // AMOR_H
