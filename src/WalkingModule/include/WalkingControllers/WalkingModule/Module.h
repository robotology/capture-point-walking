/**
 * @file WalkingModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_MODULE_HPP
#define WALKING_MODULE_HPP

// std
#include <memory>
#include <deque>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RpcClient.h>


// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// WalkingControllers library
#include <WalkingControllers/RobotInterface/Helper.h>
#include <WalkingControllers/RobotInterface/PIDHandler.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>
#include <WalkingControllers/TrajectoryPlanner/StableDCMModel.h>
#include <WalkingControllers/TrajectoryPlanner/FreeSpaceEllipseManager.h>

#include <WalkingControllers/SimplifiedModelControllers/DCMModelPredictiveController.h>
#include <WalkingControllers/SimplifiedModelControllers/DCMReactiveController.h>
#include <WalkingControllers/SimplifiedModelControllers/ZMPController.h>

#include <WalkingControllers/WholeBodyControllers/InverseKinematics.h>
#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics.h>
#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics_osqp.h>
#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics_qpOASES.h>

#include <WalkingControllers/KinDynWrapper/Wrapper.h>

#include <WalkingControllers/RetargetingHelper/Helper.h>

#include <WalkingControllers/LoggerClient/LoggerClient.h>

#include <WalkingControllers/TimeProfiler/TimeProfiler.h>

// iCub-ctrl
#include <iCub/ctrl/filters.h>

#include <thrifts/WalkingCommands.h>

namespace WalkingControllers
{

/**
 * RFModule of the Walking controller
 */
    class WalkingModule: public yarp::os::RFModule, public WalkingCommands
    {
        enum class WalkingFSM {Idle, Configured, Preparing, Prepared, Walking, Paused, Stopped};
        WalkingFSM m_robotState{WalkingFSM::Idle}; /**< State  of the WalkingFSM. */

        double m_dT; /**< RFModule period. */
        double m_time; /**< Current time. */
        std::string m_robot; /**< Robot name. */

        bool m_useMPC; /**< True if the MPC controller is used. */
        bool m_useQPIK; /**< True if the QP-IK is used. */
        bool m_useOSQP; /**< True if osqp is used to QP-IK problem. */
        bool m_dumpData; /**< True if data are saved. */
        bool m_firstRun; /**< True if it is the first run. */

        double m_maxInitialCoMVelocity; /**< Bound on the initial CoM velocity to check if the robot is going to jump at startup. */

        iDynTree::Vector2 m_previousZMP; /**< Previous ZMP value to check if the ZMP was constant for a while. */
        int m_constantZMPCounter; /**< Counter to check for how long the ZMP was constant. */
        double m_constantZMPTolerance; /**< Tolerance to consider the ZMP constant. */
        int m_constantZMPMaxCounter; /**< Max counter value for triggering the error on the constant measured ZMP. */
        double m_minimumNormalForceZMP; /**< Minimum force value to consider a contact stable. */
        iDynTree::Vector2 m_maxZMP; /**< Max value to consider the local ZMP valid. */

        std::unique_ptr<RobotInterface> m_robotControlHelper; /**< Robot control helper. */
        std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator; /**< Pointer to the trajectory generator object. */
        std::unique_ptr<FreeSpaceEllipseManager> m_freeSpaceEllipseManager; /**< Pointer to the free space ellipse manager. */
        std::unique_ptr<WalkingController> m_walkingController; /**< Pointer to the walking DCM MPC object. */
        std::unique_ptr<WalkingDCMReactiveController> m_walkingDCMReactiveController; /**< Pointer to the walking DCM reactive controller object. */
        std::unique_ptr<WalkingZMPController> m_walkingZMPController; /**< Pointer to the walking ZMP controller object. */
        std::unique_ptr<WalkingIK> m_IKSolver; /**< Pointer to the inverse kinematics solver. */
        std::unique_ptr<WalkingQPIK> m_QPIKSolver; /**< Pointer to the inverse kinematics solver. */
        std::unique_ptr<WalkingFK> m_FKSolver; /**< Pointer to the forward kinematics solver. */
        std::unique_ptr<StableDCMModel> m_stableDCMModel; /**< Pointer to the stable DCM dynamics. */
        std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */
        std::unique_ptr<RetargetingClient> m_retargetingClient; /**< Pointer to the stable DCM dynamics. */
        std::unique_ptr<LoggerClient> m_walkingLogger; /**< Pointer to the Walking Logger object. */
        std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

        double m_additionalRotationWeightDesired; /**< Desired additional rotational weight matrix. */
        double m_desiredJointsWeight; /**< Desired joint weight matrix. */
        yarp::sig::Vector m_desiredJointInRadYarp; /**< Desired joint position (regularization task). */

        std::deque<iDynTree::Transform> m_leftTrajectory; /**< Deque containing the trajectory of the left foot. */
        std::deque<iDynTree::Transform> m_rightTrajectory; /**< Deque containing the trajectory of the right foot. */

        std::deque<iDynTree::Twist> m_leftTwistTrajectory; /**< Deque containing the twist trajectory of the left foot. */
        std::deque<iDynTree::Twist> m_rightTwistTrajectory; /**< Deque containing the twist trajectory of the right foot. */

        std::deque<iDynTree::Vector2> m_DCMPositionDesired; /**< Deque containing the desired DCM position. */
        std::deque<iDynTree::Vector2> m_DCMVelocityDesired; /**< Deque containing the desired DCM velocity. */
        std::deque<bool> m_leftInContact; /**< Deque containing the left foot state. */
        std::deque<bool> m_rightInContact; /**< Deque containing the right foot state. */
        std::deque<double> m_comHeightTrajectory; /**< Deque containing the CoM height trajectory. */
        std::deque<double> m_comHeightVelocity; /**< Deque containing the CoM height velocity. */
        std::deque<size_t> m_mergePoints; /**< Deque containing the time position of the merge points. */
        std::deque<bool> m_isStancePhase; /**< if true the robot is not walking */

        std::deque<bool> m_isLeftFixedFrame; /**< Deque containing when the main frame of the left foot is the fixed frame
                                                In general a main frame of a foot is the fix frame only during the
                                                stance and the switch out phases. */


        iDynTree::ModelLoader m_loader; /**< Model loader class. */

        iDynTree::VectorDynSize m_qDesired; /**< Vector containing the results of the IK algorithm [rad]. */
        iDynTree::VectorDynSize m_dqDesired; /**< Vector containing the results of the IK algorithm [rad]. */

        iDynTree::Rotation m_inertial_R_worldFrame; /**< Rotation between the inertial and the world frame. */

        yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */
        yarp::os::BufferedPort<yarp::sig::Vector> m_desiredUnyciclePositionPort; /**< Desired robot position port. */

        bool m_newTrajectoryRequired; /**< if true a new trajectory will be merged soon. (after m_newTrajectoryMergeCounter - 2 cycles). */
        size_t m_newTrajectoryMergeCounter; /**< The new trajectory will be merged after m_newTrajectoryMergeCounter - 2 cycles. */

        double m_rootLinkOffset{0};
        
        std::mutex m_mutex; /**< Mutex. */

        iDynTree::Vector2 m_desiredPosition;

        // debug
        std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};

        /**
         * Get the robot model from the resource finder and set it.
         * @param rf is the reference to a resource finder object.
         * @return true in case of success and false otherwise.
         */
        bool setRobotModel(const yarp::os::Searchable& rf);

        /**
         * Propagate time.
         */
        void propagateTime();

        /**
         * Advance the reference signal.
         * @return true in case of success and false otherwise.
         */
        bool advanceReferenceSignals();


        /**
         * Update the FK solver.
         * @return true in case of success and false otherwise.
         */
        bool updateFKSolver();

        /**
         * Set the QP-IK problem.
         * @param solver is the pointer to the solver (osqp or qpOASES)
         * @param desiredCoMPosition desired CoM position;
         * @param desiredCoMVelocity desired CoM velocity;
         * @param desiredNeckOrientation desired neck orientation (rotation matrix);
         * @param output is the output of the solver (i.e. the desired joint velocity)
         * @return true in case of success and false otherwise.
         */
        bool solveQPIK(const std::unique_ptr<WalkingQPIK>& solver,
                       const iDynTree::Position& desiredCoMPosition,
                       const iDynTree::Vector3& desiredCoMVelocity,
                       const iDynTree::Rotation& desiredNeckOrientation,
                       iDynTree::VectorDynSize &output);

        /**
         * Evaluate the position of Zero momentum point.
         * @param zmp zero momentum point.
         * @return true in case of success and false otherwise.
         */
        bool evaluateZMP(iDynTree::Vector2& zmp);

        /**
         * Generate the first trajectory.
         * This method has to be called before updateTrajectories() method.
         * @return true in case of success and false otherwise.
         */
        bool generateFirstTrajectories();

        /**
         * Generate the first trajectory. (onTheFly)
         * @param leftToRightTransform transformation between left and right feet.
         * @return true in case of success and false otherwise.
         */
        bool generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform);

        /**
         * Ask for a new trajectory (The trajectory will be evaluated by a thread).
         * @param initTime is the initial time of the trajectory;
         * @param isLeftSwinging todo wrong name?;
         * @param measuredTransform transformation between the world and the (stance/swing??) foot;
         * @param mergePoint is the instant at which the old and the new trajectory will be merged;
         * @param desiredPosition final desired position of the projection of the CoM.
         * @return true/false in case of success/failure.
         */
        bool askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                const iDynTree::Transform& measuredTransform,
                                const size_t& mergePoint, const iDynTree::Vector2& desiredPosition);

        /**
         * Update the old trajectory.
         * This method has to be called only if the trajectory generator has finished to evaluate the new trajectory.
         * The old and the new trajectory will be merged at mergePoint.
         * @param mergePoint instant at which the old and the new trajectory will be merged
         * @return true/false in case of success/failure.
         */
        bool updateTrajectories(const size_t& mergePoint);

        /**
         * Set the input of the planner. The desired position is expressed using a
         * reference frame attached to the robot. The X axis points forward while the
         * Y axis points on the left.
         * @param x desired forward position of the robot
         * @param y desired lateral position of the robot
         * @return true/false in case of success/failure.
         */
        bool setPlannerInput(double x, double y);

        /**
         * Reset the entire controller architecture
         */
        void reset();

    public:

        /**
         * Get the period of the RFModule.
         * @return the period of the module.
         */
        double getPeriod() override;

        /**
         * Main function of the RFModule.
         * @return true in case of success and false otherwise.
         */
        bool updateModule() override;

        /**
         * Configure the RFModule.
         * @param rf is the reference to a resource finder object
         * @return true in case of success and false otherwise.
         */
        bool configure(yarp::os::ResourceFinder& rf) override;

        /**
         * Close the RFModule.
         * @return true in case of success and false otherwise.
         */
        bool close() override;

        /**
         * This allows you to put the robot in a home position for walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool prepareRobot(bool onTheFly = false) override;

        /**
         * Start walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool startWalking() override;

        /**
         * set the desired final position of the CoM.
         * @param x desired x position of the CoM;
         * @param y desired y position of the CoM.
         * @return true in case of success and false otherwise.
         */
        virtual bool setGoal(double x, double y) override;

        /**
         * Pause walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool pauseWalking() override;

        /**
         * Stop walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool stopWalking() override;
    };
};
#endif
