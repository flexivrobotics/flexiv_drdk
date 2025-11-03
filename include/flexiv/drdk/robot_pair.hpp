/**
 * @file robot_pair.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DRDK_ROBOT_PAIR_HPP_
#define FLEXIV_DRDK_ROBOT_PAIR_HPP_

#include <flexiv/rdk/robot.hpp>
#include <memory>
#include <map>

namespace flexiv {
namespace drdk {

using namespace rdk;

/**
 * @class RobotPair
 * @brief Main interface to simultaneously control a pair of robots. The commands and requests are
 * sent to both robots in parallel.
 */
class RobotPair
{
public:
    /**
     * @brief [Blocking] Instantiate the control interface of the robot pair. Background services
     * will be started and establish connection with the target robots.
     * @param[in] robots_sn Serial numbers of the left and right robot respectively. The accepted
     * formats are: "Rizon 4s-123456" and "Rizon4s-123456".
     * @param[in] translations_in_world Respective translations (x, y, z) of both robots' base
     * frames with regard to a shared world frame. To configure orientations, please use the Robot
     * Mounting setting in Flexiv Elements (under Settings -> Basic Safety Configuration). If left
     * empty, the shared world frame coincides with both robots' base frame.
     * @param[in] network_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the specified robots. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for specified robots.
     * @throw std::invalid_argument if the format of any provided robot serial number is invalid.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if either of the connected robots lacks a valid RDK license, or is
     * incompatible with this DRDK library version, or is an unsupported robot model.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with both robots is established.
     */
    RobotPair(const std::pair<std::string, std::string>& robots_sn,
        const std::pair<std::array<double, 3>, std::array<double, 3>>& translations_in_world = {},
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~RobotPair();

    //========================================= ACCESSORS ==========================================
    /**
     * @brief [Non-blocking] Whether the connection with both robots in the pair is established.
     * @return True: both connected; false: one or both disconnected.
     */
    bool connected() const;

    /**
     * @brief [Non-blocking] General information about both robots in the pair.
     * @return Respective value copies of rdk::RobotInfo struct.
     */
    std::pair<RobotInfo, RobotInfo> info() const;

    /**
     * @brief [Non-blocking] Current control mode of both robots in the pair.
     * @return Respective rdk::Mode enums.
     */
    std::pair<Mode, Mode> mode() const;

    /**
     * @brief [Non-blocking] Current states data of both robots in the pair.
     * @return Respective value copies of rdk::RobotStates struct.
     */
    std::pair<RobotStates, RobotStates> states() const;

    /**
     * @brief [Non-blocking] Whether both robots in the pair have come to a complete stop.
     * @return True: both stopped; false: one or both are still moving.
     */
    bool stopped() const;

    /**
     * @brief [Non-blocking] Whether both robots in the pair are ready to be operated, which
     * requires the following conditions to be met: enabled, brakes fully released, in auto mode, no
     * fault, and not in reduced state.
     * @return True: both operational (operational_status() == READY); false: one or both are not
     * operational.
     * @warning The robots won't execute any user command until both are ready to be operated.
     */
    bool operational() const;

    /**
     * @brief [Non-blocking] Current operational status of both robots in the pair.
     * @return Respective rdk::OperationalStatus enums.
     */
    std::pair<OperationalStatus, OperationalStatus> operational_status() const;

    /**
     * @brief [Non-blocking] Whether any robot in the pair is busy. This includes any user commanded
     * operations that requires the robot to execute. For example, plans, primitives, Cartesian and
     * joint motions, etc.
     * @return True: either robot is busy; false: both robots are idle.
     * @warning Some exceptions exist for primitives, see ExecutePrimitive() for more details.
     */
    bool busy() const;

    /**
     * @brief [Non-blocking] Which robot in the pair is busy.
     * @return True: this robot is busy; false: this robot is idle.
     */
    std::pair<bool, bool> which_busy() const;

    /**
     * @brief [Non-blocking] Whether any robot in the pair is in fault state.
     * @return True: either robot has fault; false: both robots are normal.
     */
    bool fault() const;

    /**
     * @brief [Non-blocking] Which robot in the pair is in fault state.
     * @return True: this robot has fault; false: this robot is normal.
     */
    std::pair<bool, bool> which_fault() const;

    /**
     * @brief [Non-blocking] Whether a robot in the pair is in reduced state.
     * @return True: this robot is in reduced state; false: this robot is not in reduced state.
     * @par Reduced state
     * The robot will enter reduced state if a) the safety input for reduced state goes low or b)
     * robot TCP passes through any safety plane. The safety limits are lowered in reduced state
     * compared to normal state. Specific values for the safety limits can be configured in Flexiv
     * Elements under Settings -> Safety Configuration. Please refer to the robot user manual for
     * more details about system reduced state.
     */
    std::pair<bool, bool> reduced() const;

    /**
     * @brief [Non-blocking] Whether a robot in the pair is in recovery state.
     * @return True: this robot is in recovery state; false: this robot is not in recovery state.
     * @note Use RunAutoRecovery() to execute automatic recovery operation.
     * @par Recovery state
     * The robot will enter recovery state if it needs to recover from joint position limit
     * violation (a critical system fault that requires a recovery operation, during which the
     * joints that moved outside the allowed position range will need to move very slowly back into
     * the allowed range). Please refer to the robot user manual for more details about system
     * recovery state.
     */
    std::pair<bool, bool> recovery() const;

    /**
     * @brief [Non-blocking] Whether the emergency stop of a robot in the pair is released.
     * @return True: this robot's E-Stop is released; false: this robot's E-Stop is pressed.
     */
    std::pair<bool, bool> estop_released() const;

    /**
     * @brief [Non-blocking] Whether the enabling button of a robot in the pair is pressed.
     * @return True: this robot's enabling button is pressed; false: this robot's enabling button is
     * released.
     */
    std::pair<bool, bool> enabling_button_pressed() const;

    /**
     * @brief [Non-blocking] Robot events stored since the last successful instantiation of this
     * class.
     * @return Respective event logs as vectors of RobotEvent struct, with the first vector element
     * being the oldest event and the last vector element being the latest event.
     * @warning Events before the last successful instantiation of this class are not stored.
     */
    std::pair<std::vector<RobotEvent>, std::vector<RobotEvent>> event_log() const;

    /**
     * @brief [Non-blocking] Pointers to the underlying rdk::Robot instances of the robot pair.
     * @return Respective pointers to rdk::Robot instances.
     */
    std::pair<std::shared_ptr<Robot>, std::shared_ptr<Robot>> instances() const;

    //======================================= SYSTEM CONTROL =======================================
    /**
     * @brief [Blocking] Enable both robots in the pair. If E-stop is released and there's no fault,
     * the robots will release brakes, and becomes operational a few seconds later.
     * @throw std::logic_error if either robot is not connected.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     */
    void Enable();

    /**
     * @brief [Blocking] Force the brakes of both robots in the pair to engage or release during
     * normal operation. Restrictions apply, see warning.
     * @param[in] engage True: engage brakes; false: release brakes.
     * @throw std::logic_error if either robot is not a medical model or is moving.
     * @throw std::runtime_error if failed to engage/release the brakes.
     * @note This function blocks until the brakes are successfully engaged/released.
     * @warning This function is accessible only if a) both robots are medical models AND
     * b) both robots are not moving.
     */
    void Brake(bool engage);

    /**
     * @brief [Blocking] Switch both robots in the pair to new control modes and wait for the mode
     * transition to finish.
     * @param[in] modes Respective rdk::Mode enums.
     * @throw std::invalid_argument if the requested mode is invalid or unlicensed.
     * @throw std::logic_error if either robot is in an unknown control mode or is not operational.
     * @throw std::runtime_error if failed to transit either robot into the specified control mode
     * after several attempts.
     * @note This function blocks until both robots have successfully transited into the specified
     * control modes.
     * @warning If either robot is still moving when this function is called, it will automatically
     * stop before making the mode transition.
     */
    void SwitchMode(const std::pair<Mode, Mode>& modes);

    /**
     * @overload Switch both robots in the pair to the same new control mode.
     */
    void SwitchMode(Mode mode);

    /**
     * @brief [Blocking] Stop one robot in the pair and transit its control mode to IDLE.
     * @param[in] mask True: stop this robot; false: skip this robot.
     * @throw std::runtime_error if failed to stop the robot.
     * @note This function blocks until the robot comes to a complete stop.
     */
    void Stop(std::pair<bool, bool> mask);

    /**
     * @overload Stop both robots in the pair and transit their control modes to IDLE.
     */
    void Stop();

    /**
     * @brief [Blocking] For both robots in the pair, try to clear minor or critical fault without a
     * power cycle.
     * @param[in] timeout_sec Maximum time in seconds to wait for the fault to be successfully
     * cleared. Normally, a minor fault should take no more than 3 seconds to clear, and a critical
     * fault should take no more than 30 seconds to clear.
     * @return True: cleared fault on this robot; false: failed to clear fault on this robot.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the faults on both robots are successfully cleared or
     * [timeout_sec] has elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    std::pair<bool, bool> ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Blocking] Run automatic recovery to bring joints of either robot that are outside the
     * allowed position range back into allowed range.
     * @throw std::runtime_error if failed to enter automatic recovery mode.
     * @note Refer to user manual for more details.
     * @note This function blocks until the automatic recovery process is finished.
     * @see recovery().
     */
    void RunAutoRecovery();

    /**
     * @brief [Blocking] Set values to global variables that already exist in the two robots.
     * @param[in] global_vars Respective maps of {global_var_name, global_var_value(s)}. Use int 1
     * and 0 to represent booleans. For example, {{"camera_offset", {0.1, -0.2, 0.3}},
     * {"start_plan", 1}}.
     * @throw std::length_error if [global_vars] is too long to transmit in one request.
     * @throw std::invalid_argument if any of the specified global variables does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the global variables are successfully set.
     * @warning The specified global variables need to be created first using Flexiv Elements.
     * @see global_variables().
     */
    void SetGlobalVariables(const std::pair<std::map<std::string, FlexivDataTypes>,
        std::map<std::string, FlexivDataTypes>>& global_vars);

    /**
     * @brief [Blocking] Existing global variables and their current values from the robot pair.
     * @return Respective maps of {global_var_name, global_var_value(s)}. Booleans are represented
     * by int 1 and 0. For example, {{"camera_offset", {0.1, -0.2, 0.3}}, {"start_plan", 1}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot pair.
     * @note This function blocks until a reply is received.
     * @see SetGlobalVariables().
     */
    std::pair<std::map<std::string, FlexivDataTypes>, std::map<std::string, FlexivDataTypes>>
    global_variables() const;

    /**
     * @brief [Blocking] Lock/unlock external axes (if any) of the robot pair during primitive
     * execution, direct joint control, and direct Cartesian control modes.
     * @param[in] toggles True: the external axes of this robot are locked and will not move; false:
     * the external axes of this robot are not locked and will move. By default, the external axes
     * are locked for both robots.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     */
    void LockExternalAxes(std::pair<bool, bool> toggles);

    /**
     * @brief [Blocking] Sync/unsync TCP motion of the robot pair with the movement of the
     * positioners (if any) during primitive execution.
     * @param[in] toggles True: the motion sync for this robot is on; false: motion sync for this
     * robot is off. By default, the motion sync is off for both robots.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Only applicable to certain primitives that support motion sync with the positioner.
     * @note This function blocks until the request is successfully delivered.
     */
    void SyncWithPositioner(std::pair<bool, bool> toggles);

    //======================================= PLAN EXECUTION =======================================
    /**
     * @brief [Blocking] Execute plans simultaneously on both robots in the pair by specifying plan
     * indices.
     * @param[in] indices Respective indices of the plans to execute, can be obtained via
     * plan_list().
     * @param[in] continue_exec Whether to continue executing the plan when the DRDK program is
     * closed or the connection is lost.
     * @param[in] block_until_started Whether to wait for the commanded plan to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the plan ready, the loading process typically takes no more than 200 ms.
     * @throw std::invalid_argument if either element in [indices] is outside the valid range.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the plan has started execution if
     * [block_until_started] is enabled.
     * @note busy() can be used to check if the plan has finished.
     */
    void ExecutePlan(std::pair<unsigned int, unsigned int> indices, bool continue_exec = false,
        bool block_until_started = true);

    /**
     * @brief [Blocking] Execute plans simultaneously on both robots in the pair by specifying plan
     * names.
     * @param[in] names Respective names of the plans to execute, can be obtained via plan_list().
     * @param[in] continue_exec Whether to continue executing the plan when the DRDK program is
     * closed or the connection is lost.
     * @param[in] block_until_started Whether to wait for the commanded plan to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the plan ready, the loading process typically takes no more than 200 ms.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the plan has started execution if
     * [block_until_started] is enabled.
     * @note busy() can be used to check if the plan has finished.
     */
    void ExecutePlan(const std::pair<std::string, std::string>& names, bool continue_exec = false,
        bool block_until_started = true);

    /**
     * @brief [Blocking] Pause or resume the execution of current plans on both robots in the pair.
     * @param[in] toggles True: pause plan on this robot; false: resume plan on this robot.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     * @warning Internal plans (not created by user) cannot be resumed due to safety concerns.
     */
    void PausePlan(std::pair<bool, bool> toggles);

    /**
     * @brief [Blocking] Stop one robot in the pair and transit its control mode to IDLE.
     * @param[in] mask True: stop this robot; false: skip this robot.
     * @throw std::runtime_error if failed to stop the robot.
     * @note This function blocks until the robot comes to a complete stop.
     */
    /**
     * @brief [Blocking] Stop the execution of the current plan for one robot in the pair.
     * @param[in] mask True: stop plan for this robot; false: skip this robot.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     */
    void StopPlan(std::pair<bool, bool> mask);

    /**
     * @overload Stop the execution of the current plan for both robots in the pair.
     */
    void StopPlan();

    /**
     * @brief [Blocking] Lists of all available plans from both robots in the pair.
     * @return Respective available plans as string lists.
     * @throw std::runtime_error if failed to get a reply from the connected robot pair.
     * @note This function blocks until a reply is received.
     */
    std::pair<std::vector<std::string>, std::vector<std::string>> plan_list() const;

    /**
     * @brief [Blocking] Detailed information about the executing plans on both robots in the pair.
     * Contains plan name, primitive name, node name, node path, node path time period, etc.
     * @return Respective value copies of rdk::PlanInfo struct.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to get a reply from the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until a reply is received.
     */
    std::pair<PlanInfo, PlanInfo> plan_info() const;

    /**
     * @brief [Blocking] Enable or disable the breakpoint mode during plan execution. When enabled,
     * the executing plan will pause at the pre-defined breakpoints. Use StepBreakpoint() to
     * continue the execution and pause at the next breakpoint.
     * @param[in] is_enabled True: enable on this robot; false: disable on this robot. By default,
     * breakpoint mode is disabled on both robots in the pair.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetBreakpointMode(std::pair<bool, bool> is_enabled);

    /**
     * @brief [Blocking] If breakpoint mode is enabled, step to the next breakpoint. The plan
     * execution will continue and pause at the next breakpoint.
     * @param[in] step_mask True: step this robot; false: skip this robot.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     * @note Use PlanInfo::waiting_for_step to check if the plan is waiting for user signal to step
     * the breakpoint.
     */
    void StepBreakpoint(std::pair<bool, bool> step_mask);

    /**
     * @brief [Blocking] For both robots in the pair, set overall velocity scale of robot motions
     * during plan and primitive execution.
     * @param[in] velocity_scales Respective percentage scales to adjust the overall velocity of
     * robot motions. Valid range: [0, 100]. Setting to 100 means to move with 100% of specified
     * motion velocity, and 0 means not moving at all.
     * @throw std::invalid_argument if either element in [velocity_scales] is outside the valid
     * range.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PLAN_EXECUTION, NRT_PRIMITIVE_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetVelocityScale(std::pair<unsigned int, unsigned int> velocity_scales);

    //==================================== PRIMITIVE EXECUTION =====================================
    /**
     * @brief [Blocking] Execute primitives simultaneously on both robots in the pair by specifying
     * primitive names and parameters, which can be found in the [Flexiv Primitives
     * documentation](https://www.flexiv.com/primitives/).
     * @param[in] primitive_names Respective primitive names. For example, "Home", "MoveL",
     * "ZeroFTSensor", etc.
     * @param[in] input_params Respective maps of {input_parameter_name, input_parameter_value(s)}
     * specifying basic and advanced parameters of the primitives. Use int 1 and 0 to represent
     * booleans. E.g. {{"target", rdk::Coord({0.65, -0.3, 0.2}, {180, 0, 180}, {"WORLD",
     * "WORLD_ORIGIN"})}, {"vel", 0.6}, {"zoneRadius", "Z50"}}.
     * @param[in] block_until_started Whether to wait for the commanded primitive to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the primitive ready, the loading process typically takes no more than 200 ms.
     * @throw std::length_error if [input_params] is too long to transmit in one request.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: NRT_PRIMITIVE_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the primitive has started execution if
     * [block_until_started] is enabled.
     * @warning The primitive input parameters may not use SI units, please refer to the Flexiv
     * Primitives documentation for exact unit definition.
     * @warning Most primitives won't exit by themselves and require users to explicitly trigger
     * transitions based on specific primitive states. In such case, busy() will stay true even if
     * it seems everything is done for that primitive.
     */
    void ExecutePrimitive(const std::pair<std::string, std::string>& primitive_names,
        const std::pair<std::map<std::string, FlexivDataTypes>,
            std::map<std::string, FlexivDataTypes>>& input_params,
        bool block_until_started = true);

    /**
     * @brief [Blocking] Names and values of the executing primitive's state parameters on both
     * robots in the pair.
     * @return Respective maps of {pt_state_name, pt_state_value(s)}. Booleans are represented by
     * int 1 and 0. E.g. {{"reachedTarget", 1}, {"timePeriod", 5.6}, {"forceOffset", {0.1, 0.2,
     * -1.3}}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot pair.
     * @note This function blocks until a reply is received.
     */
    std::pair<std::map<std::string, FlexivDataTypes>, std::map<std::string, FlexivDataTypes>>
    primitive_states() const;

    //==================================== DIRECT JOINT CONTROL ====================================
    /**
     * @brief [Non-blocking] Continuously stream joint torque commands to both robots in the pair.
     * @param[in] torques Respective target joint torques: \f$ {\tau_J}_d \in \mathbb{R}^{n \times
     * 1} \f$ for each robot. Unit: \f$ [Nm] \f$.
     * @param[in] enable_gravity_comp Enable/disable robot gravity compensation for this robot.
     * @param[in] enable_soft_limits Enable/disable soft limits for this robot to keep the joints
     * from moving outside allowed position range, which will trigger a safety fault that requires
     * recovery operation.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_JOINT_TORQUE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     */
    void StreamJointTorque(const std::pair<std::vector<double>, std::vector<double>>& torques,
        std::pair<bool, bool> enable_gravity_comp = {true, true},
        std::pair<bool, bool> enable_soft_limits = {true, true});

    /**
     * @brief [Non-blocking] Continuously stream joint position, velocity, and acceleration commands
     * to both robots in the pair. The commands are tracked by either the joint impedance controller
     * or the joint position controller, depending on the control mode.
     * @param[in] positions Respective target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1}
     * \f$ for each robot. Unit: \f$ [rad] \f$.
     * @param[in] velocities Respective target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n
     * \times 1} \f$ for each robot. Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Respective target joint accelerations: \f$ \ddot{q}_d \in
     * \mathbb{R}^{n \times 1} \f$ for each robot. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, RT_JOINT_POSITION.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     * @see SetJointImpedance().
     */
    void StreamJointPosition(const std::pair<std::vector<double>, std::vector<double>>& positions,
        const std::pair<std::vector<double>, std::vector<double>>& velocities,
        const std::pair<std::vector<double>, std::vector<double>>& accelerations);

    /**
     * @brief [Non-blocking] Discretely send joint position and velocity commands to both robots in
     * the pair. The robot's internal motion generator will smoothen the discrete commands, which
     * are tracked by either the joint impedance controller or the joint position controller,
     * depending on the control mode.
     * @param[in] positions Respective target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1}
     * \f$ for each robot. Unit: \f$ [rad] \f$.
     * @param[in] velocities Respective target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n
     * \times 1} \f$ for each robot. Each joint will maintain this amount of velocity when it
     * reaches the target position. Unit: \f$ [rad/s] \f$.
     * @param[in] max_vel Respective maximum joint velocities for the planned trajectory: \f$
     * \dot{q}_{max} \in \mathbb{R}^{n \times 1} \f$ for each robot. Unit: \f$ [rad/s] \f$.
     * @param[in] max_acc Respective maximum joint accelerations for the planned trajectory: \f$
     * \ddot{q}_{max} \in \mathbb{R}^{n \times 1} \f$ for each robot. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF, or
     * [max_vel] or [max_acc] contains any non-positive value.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @note Applicable control modes: NRT_JOINT_IMPEDANCE, NRT_JOINT_POSITION.
     * @warning Calling this function a second time while the motion from the previous call is still
     * ongoing will trigger an online re-planning of the joint trajectory, such that the previous
     * command is aborted and the new command starts to execute.
     * @see SetJointImpedance().
     */
    void SendJointPosition(const std::pair<std::vector<double>, std::vector<double>>& positions,
        const std::pair<std::vector<double>, std::vector<double>>& velocities,
        const std::pair<std::vector<double>, std::vector<double>>& max_vel,
        const std::pair<std::vector<double>, std::vector<double>>& max_acc);

    /**
     * @brief [Blocking] For both robots in the pair, set impedance properties of the joint motion
     * controller used in joint impedance control modes.
     * @param[in] K_q Respective joint motion stiffness: \f$ K_q \in \mathbb{R}^{n \times 1} \f$ for
     * each robot. Setting motion stiffness of a joint axis to 0 will make this axis free-floating.
     * Valid range: [0, RobotInfo::K_q_nom]. Unit: \f$ [Nm/rad] \f$.
     * @param[in] Z_q Respective joint motion damping ratio: \f$ Z_q \in \mathbb{R}^{n \times 1}
     * \f$ for each robot. Valid range: [0.3, 0.8]. The default value 0.7 will be used if left
     * empty.
     * @throw std::invalid_argument if [K_q] or [Z_q] contains any value outside the valid range or
     * size of any input vector does not match robot DoF.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, NRT_JOINT_IMPEDANCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning Changing damping ratio [Z_q] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetJointImpedance(const std::pair<std::vector<double>, std::vector<double>>& K_q,
        const std::pair<std::vector<double>, std::vector<double>>& Z_q = {});

    /**
     * @brief [Blocking] For both robots in the pair, set maximum contact torques for the joint
     * motion controller used in the joint impedance control modes. The controller will regulate its
     * output to maintain contact torques with the environment under the set values.
     * @param[in] max_torques Respective maximum contact torques: \f$ tau_q \in \mathbb{R}^{n \times
     * 1} \f$ for each robot. Valid range: [0, RobotInfo::tau_max]. Unit: \f$ [Nm] \f$.
     * @throw std::invalid_argument if [max_torques] contains any value outside the valid range or
     * its size does not match robot DoF.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, NRT_JOINT_IMPEDANCE.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetMaxContactTorque(
        const std::pair<std::vector<double>, std::vector<double>>& max_torques);

    /**
     * @brief [Blocking] For both robots in the pair, set inertia shaping scales for the joint
     * motion controller used in the joint impedance control modes.
     * @param[in] inertia_scales Respective inertia shaping scales: \f$ \sigma_q \in \mathbb{R}^{n
     * \times 1} \f$ for each robot. Valid range: [0.75, 1.0]. The nominal (safe) value is 1.0,
     * which means no shaping.
     * @throw std::invalid_argument if [inertia_scales] contains any value outside the valid range
     * or its size does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, NRT_JOINT_IMPEDANCE.
     * @note This function blocks until the request is successfully delivered.
     * @par Joint inertia shaping
     * In joint impedance control modes, it is possible to shape down the natural inertia of the
     * joints to make them behave as if they are lighter. The parameter [inertia_scales] sets the
     * scale of shaped/natural inertia for each joint. Smaller scale corresponds to lighter inertia.
     */
    void SetJointInertiaScale(
        const std::pair<std::vector<double>, std::vector<double>>& inertia_scales);

    //================================== DIRECT CARTESIAN CONTROL ==================================
    /**
     * @brief [Non-blocking] Discretely send Cartesian motion and/or force command to both robots in
     * the pair for them to track using its unified motion-force controller, which allows doing
     * force control in zero or more Cartesian axes and motion control in the rest axes. The robot's
     * internal motion generator will smoothen the discrete commands.
     * @param[in] poses Respective target TCP poses in world frame: \f$ {^{O}T_{TCP}}_{d} \in
     * \mathbb{R}^{7 \times 1} \f$ for each robot. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T
     * \f$. Unit: \f$ [m]:[] \f$.
     * @param[in] wrenches Respective target TCP wrenches (force and moment) in the force control
     * reference frame (configured by SetForceControlFrame()): \f$ ^{0}F_d \in \mathbb{R}^{6 \times
     * 1} \f$ for each robot. The robot will track the target wrench using an explicit force
     * controller. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1}
     * \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     * @param[in] max_linear_vel Respective maximum Cartesian linear velocities when moving to the
     * target poses. A safe value is provided as default. Unit: \f$ [m/s] \f$.
     * @param[in] max_angular_vel Respective maximum Cartesian angular velocities when moving to the
     * target poses. A safe value is provided as default. Unit: \f$ [rad/s] \f$.
     * @param[in] max_linear_acc Respective maximum Cartesian linear accelerations when moving to
     * the target poses. A safe value is provided as default. Unit: \f$ [m/s^2] \f$.
     * @param[in] max_angular_acc Respective maximum Cartesian angular accelerations when moving to
     * the target poses. A safe value is provided as default. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if any of the last 4 input parameters is negative.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @note Applicable control modes: NRT_CARTESIAN_MOTION_FORCE, NRT_SUPER_PRIMITIVE.
     * @warning Same as Flexiv Elements, the target wrench is expressed as wrench sensed at TCP
     * instead of wrench exerted by TCP. E.g. commanding f_z = +5 N will make the end-effector move
     * towards -Z direction, so that upon contact, the sensed force will be +5 N.
     * @par How to achieve pure motion control?
     * Use SetForceControlAxis() to disable force control for all Cartesian axes to achieve pure
     * motion control. This function does pure motion control by default.
     * @par How to achieve pure force control?
     * Use SetForceControlAxis() to enable force control for all Cartesian axes to achieve pure
     * force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use SetForceControlAxis() to enable force control for one or more Cartesian axes and leave
     * the rest axes motion-controlled, then provide target pose for the motion-controlled axes and
     * target wrench for the force-controlled axes.
     * @see SetCartesianImpedance(), SetMaxContactWrench(), SetNullSpacePosture(),
     * SetForceControlAxis(), SetForceControlFrame(), SetPassiveForceControl().
     */
    void SendCartesianMotionForce(
        const std::pair<std::array<double, kPoseSize>, std::array<double, kPoseSize>>& poses,
        const std::pair<std::array<double, kCartDoF>, std::array<double, kCartDoF>>& wrenches = {},
        std::pair<double, double> max_linear_vel = {0.5, 0.5},
        std::pair<double, double> max_angular_vel = {1.0, 1.0},
        std::pair<double, double> max_linear_acc = {2.0, 2.0},
        std::pair<double, double> max_angular_acc = {5.0, 5.0});

    /**
     * @brief [Blocking] For both robots in the pair, set impedance properties of the Cartesian
     * motion controller used in Cartesian motion-force control modes.
     * @param[in] K_x Respective Cartesian motion stiffness: \f$ K_x \in \mathbb{R}^{6 \times 1} \f$
     * for each robot. Setting motion stiffness of a motion-controlled Cartesian axis to 0 will make
     * this axis free-floating. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T
     * \f$. Valid range: [0, RobotInfo::K_x_nom]. Unit: \f$ [N/m]:[Nm/rad] \f$.
     * @param[in] Z_x Respective Cartesian motion damping ratio: \f$ Z_x \in \mathbb{R}^{6 \times 1}
     * \f$ for each robot. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear damping ratio and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular damping ratio: \f$ [\zeta_x, \zeta_y, \zeta_z,
     * \zeta_{Rx}, \zeta_{Ry}, \zeta_{Rz}]^T \f$. Valid range: [0.3, 0.8]. The nominal (safe) value
     * is provided as default.
     * @throw std::invalid_argument if [K_x] or [Z_x] contains any value outside the valid range.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @warning Changing damping ratio [Z_x] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetCartesianImpedance(
        const std::pair<std::array<double, kCartDoF>, std::array<double, kCartDoF>>& K_x,
        const std::pair<std::array<double, kCartDoF>, std::array<double, kCartDoF>>& Z_x
        = {{0.7, 0.7, 0.7, 0.7, 0.7, 0.7}, {0.7, 0.7, 0.7, 0.7, 0.7, 0.7}});

    /**
     * @brief [Blocking] For both robots in the pair, set maximum contact wrench for the motion
     * control part of the Cartesian motion-force control modes. The controller will regulate its
     * output to maintain contact wrench (force and moment) with the environment under the set
     * values.
     * @param[in] max_wrenches Respective maximum contact wrenches (force and moment): \f$ F_max \in
     * \mathbb{R}^{6 \times 1} \f$ for each robot. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * maximum force and \f$ \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     * @throw std::invalid_argument if [max_wrenches] contains any negative value.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note The maximum contact wrench regulation only applies to the motion control part.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The maximum contact wrench regulation cannot be enabled if any of the rotational
     * Cartesian axes is enabled for moment control.
     */
    void SetMaxContactWrench(
        const std::pair<std::array<double, kCartDoF>, std::array<double, kCartDoF>>& max_wrenches);

    /**
     * @brief [Blocking] For both robots in the pair, set reference joint positions for the
     * null-space posture control module used in the Cartesian motion-force control modes.
     * @param[in] ref_positions Respective reference joint positions for the null-space posture
     * control: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$ for each robot. Valid range:
     * [RobotInfo::q_min, RobotInfo::q_max]. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [ref_positions] contains any value outside the valid
     * range or size of any input vector does not match robot DoF.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The reference joint positions will be automatically reset to the robot's current
     * joint positions upon re-entering the applicable control modes.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DoF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the reference joint positions of a
     * desired robot posture are set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void SetNullSpacePosture(
        const std::pair<std::vector<double>, std::vector<double>>& ref_positions);

    /**
     * @brief [Blocking] For both robots in the pair, set weights of the three optimization
     * objectives while computing null-space postures. Change the weights to optimize robot
     * performance for different use cases.
     * @param[in] linear_manipulability Respectively increase this weight to improve the robots'
     * capability to translate freely in Cartesian space, i.e. a broader range of potential
     * translation movements. Valid range: [0.0, 1.0].
     * @param[in] angular_manipulability Respectively increase this weight to improve the robots'
     * capability to rotate freely in Cartesian space, i.e. a broader range of potential rotation
     * movements. Valid range: [0.0, 1.0].
     * @param[in] ref_positions_tracking Respectively increase this weight to make the robots track
     * closer to the reference joint positions specified using SetNullSpacePosture(). Valid range:
     * [0.1, 1.0].
     * @throw std::invalid_argument if any of the input parameters is outside its valid range.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note The default value is provided for each parameter.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The optimization weights will be automatically reset to the provided default values
     * upon re-entering the applicable control modes.
     */
    void SetNullSpaceObjectives(std::pair<double, double> linear_manipulability = {0.0, 0.0},
        std::pair<double, double> angular_manipulability = {0.0, 0.0},
        std::pair<double, double> ref_positions_tracking = {0.5, 0.5});

    /**
     * @brief [Blocking] For both robots in the pair, set Cartesian axes to enable force control
     * while in the Cartesian motion-force control modes. Axes not enabled for force control will be
     * motion-controlled.
     * @param[in] enabled_axes Respective flags to enable/disable force control for certain
     * Cartesian axes in the force control reference frame (configured by SetForceControlFrame()).
     * The axis order is \f$ [X, Y, Z, Rx, Ry, Rz] \f$.
     * @param[in] max_linear_vel For linear Cartesian axes that are enabled for force control,
     * respectively limit the moving velocities to these values as a protection mechanism in case of
     * contact loss. The axis order is \f$ [X, Y, Z] \f$. Valid range: [0.005, 2.0]. Unit: \f$ [m/s]
     * \f$.
     * @throw std::invalid_argument if [max_linear_vel] contains any value outside the valid range.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, force control is disabled for all Cartesian axes by default.
     * @warning The maximum linear velocity protection for force control axes is only effective
     * under active force control (i.e. passive force control disabled), see
     * SetPassiveForceControl().
     */
    void SetForceControlAxis(
        const std::pair<std::array<bool, kCartDoF>, std::array<bool, kCartDoF>>& enabled_axes,
        const std::pair<std::array<double, kCartDoF / 2>, std::array<double, kCartDoF / 2>>&
            max_linear_vel
        = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}});

    /**
     * @brief [Blocking] For both robots in the pair, set reference frames for force control while
     * in the Cartesian motion-force control modes. The force control frame is defined by specifying
     * its transformation with regard to the root coordinate.
     * @param[in] root_coords Respective reference coordinates of [T_in_root].
     * @param[in] T_in_root Respective transformations from [root_coords] to the user-defined force
     * control frame: \f$ ^{root}T_{force} \in \mathbb{R}^{7 \times 1} \f$. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x,
     * y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$. If root coordinate is a fixed one
     * (e.g. WORLD), then the force control frame will also be fixed; if root coordinate is a moving
     * one (e.g. TCP), then the force control frame will also be moving with the root coordinate. An
     * identity transformation is provided as default.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE,
     * NRT_SUPER_PRIMITIVE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, the robot will use WORLD origin as the force control frame by default.
     * @par Force control frame
     * In Cartesian motion-force control modes, the reference frame of motion control is always the
     * world frame, but the reference frame of force control can be an arbitrary one. While the
     * world frame is the commonly used global coordinate, the current TCP frame is a dynamic local
     * coordinate whose transformation with regard to the world frame changes as the robot TCP
     * moves. When using world frame with no transformation as the force control frame, the
     * force-controlled axes and motion-controlled axes are guaranteed to be orthogonal. Otherwise,
     * the force-controlled axes and motion-controlled axes are NOT guaranteed to be orthogonal
     * because different reference frames are used. In this case, it's recommended but not required
     * to set the target pose such that the actual robot motion direction(s) are orthogonal to force
     * direction(s). If they are not orthogonal, the motion control's vector component(s) in the
     * force direction(s) will be eliminated.
     */
    void SetForceControlFrame(std::pair<CoordType, CoordType> root_coords,
        const std::pair<std::array<double, kPoseSize>, std::array<double, kPoseSize>>& T_in_root
        = {{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}});

    /**
     * @brief [Blocking] For both robots in the pair, enable or disable passive force control for
     * the Cartesian motion-force control modes. When enabled, an open-loop force controller will be
     * used to feed forward the target wrench, i.e. passive force control. When disabled, a
     * closed-loop force controller will be used to track the target wrench, i.e. active force
     * control.
     * @param[in] is_enabled True: enable on this robot; false: disable on this robot. By default,
     * passive force control is disabled and active force control is used.
     * @throw std::logic_error if either robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, the passive force control is disabled by default.
     * @par Difference between active and passive force control
     * Active force control uses a feedback loop to reduce the error between target wrench and
     * measured wrench. This method results in better force tracking performance, but at the cost of
     * additional Cartesian damping which could potentially decrease motion tracking performance. On
     * the other hand, passive force control simply feeds forward the target wrench. This methods
     * results in worse force tracking performance, but is more robust and does not introduce
     * additional Cartesian damping. The choice of active or passive force control depends on the
     * actual application.
     */
    void SetPassiveForceControl(std::pair<bool, bool> is_enabled);

    //======================================== IO CONTROL ========================================
    /**
     * @brief [Blocking] Set one or more digital output ports on both robots in the pair. Each robot
     * has 16 ports on the control box and 2 ports inside the wrist connector.
     * @param[in] digital_outputs Respective maps of {port_index, port_value}. For [port_index], the
     * valid range is [0, 17]. For [port_value], true: set port high, false: set port low. E.g. {{1,
     * true}, {3, false}, {10, true}}.
     * @throw std::invalid_argument if any provided port index is outside the valid range.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetDigitalOutputs(
        const std::pair<std::map<unsigned int, bool>, std::map<unsigned int, bool>>&
            digital_outputs);

    /**
     * @brief [Non-blocking] Current reading from all digital input ports on both robots in the
     * pair. Each robot has 16 ports on the control box and 2 ports inside the wrist connector.
     * @return Respective boolean arrays whose index corresponds to that of the digital input ports.
     * True: port high; false: port low.
     */
    std::pair<std::array<bool, kIOPorts>, std::array<bool, kIOPorts>> digital_inputs() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    friend class GripperPair;
    friend class BimanualPrimitives;
};

} /* namespace drdk */
} /* namespace flexiv */

#endif /* FLEXIV_DRDK_ROBOT_PAIR_HPP_ */
