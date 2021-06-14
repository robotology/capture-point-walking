/**
 * @file Wrapper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_KINDYN_WRAPPER_WRAPPER_H
#define WALKING_CONTROLLERS_KINDYN_WRAPPER_WRAPPER_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

//iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

// iCub-ctrl
#include <iCub/ctrl/filters.h>

#include <unordered_map>

namespace WalkingControllers
{

    class WalkingFK
    {
        iDynTree::KinDynComputations m_kinDyn; /**< KinDynComputations solver. */

        bool m_useExternalRobotBase; /**< is external estimator for the base of robot used? */
        iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces;

        bool m_prevContactLeft; /**< Boolean is the previous contact foot the left one? */
        bool m_dcmEvaluated; /**< is the DCM evaluated? */
        bool m_comEvaluated; /**< is the CoM evaluated? */

        iDynTree::FrameIndex m_frameBaseIndex;
        iDynTree::FrameIndex m_frameLeftIndex; /**< Index of the frame attached to the left foot in which all the left foot transformations are expressed. */
        iDynTree::FrameIndex m_frameRightIndex; /**< Index of the frame attached to the right foot in which all the right foot transformations are expressed. */
        iDynTree::FrameIndex m_frameRootIndex; /**< Index of the frame attached to the root_link. */
        iDynTree::FrameIndex m_frameNeckIndex; /**< Index of the frame attached to the neck_2. */
        iDynTree::FrameIndex m_frameLeftHandIndex; /**< Index of the frame attached to the left hand. */
        iDynTree::FrameIndex m_frameRightHandIndex; /**< Index of the frame attached to the right hand. */
        iDynTree::FrameIndex m_frameHeadIndex; /**< Index of the frame attached to the head. */

        std::string m_baseFrameLeft; /**< Name of the left base frame. */
        std::string m_baseFrameRight;  /**< Name of the right base frame. */

        iDynTree::Transform m_frameHlinkLeft; /**< Transformation between the l_sole and the l_foot frame (l_ankle_2?!). */
        iDynTree::Transform m_frameHlinkRight; /**< Transformation between the l_sole and the l_foot frame (l_ankle_2?!). */
        iDynTree::Transform m_worldToBaseTransform; /**< World to base transformation. */
        std::unordered_map<std::string, std::pair<const std::string, const iDynTree::Transform>> m_baseFrames;/**< Transform related to the base frame */
        iDynTree::Twist m_baseTwist;/**< twist related to base frame */

        iDynTree::Position m_comPosition; /**< Position of the CoM. */
        iDynTree::Vector3 m_comVelocity; /**< Velocity of the CoM. */
        iDynTree::Vector2 m_dcm; /**< DCM position. */
        double m_omega; /**< Inverted time constant of the 3D-LIPM. */

        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_comPositionFilter; /**< CoM position low pass filter. */
        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_comVelocityFilter; /**< CoM velocity low pass filter. */
        iDynTree::Position m_comPositionFiltered; /**< Filtered position of the CoM. */
        iDynTree::Vector3 m_comVelocityFiltered; /**< Filtered velocity of the CoM. */
        bool m_useFilters; /**< If it is true the filters will be used. */

        bool m_firstStep; /**< True only during the first step. */

        iDynTree::VectorDynSize m_jointPositions; /**< joint positions in radians. */

        /**
         * Set the model of the robot.
         * @param model iDynTree model.
         * @return true/false in case of success/failure.
         */
        bool setRobotModel(const iDynTree::Model& model);

        /**
         * Set The base frames.
         * @note: During the walking task the frame shift from the left to the right foot.
         * In general the base link is coincident to the stance foot.
         * @param lFootFrame name of the frame attached to the left foot;
         * @param tFootFrame name of the frame attached to the right foot;
         * @return true/false in case of success/failure.
         */
        bool setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame);

        /**
         * Set The base frames for enabling using Gazebo as a base estimator.
         * @param baseFrame the frame name inside model;
         * @param name label for storing the frame information;
         * @return true/false in case of success/failure.
         */
        bool setBaseFrame(const std::string& baseFrame, const std::string& name);

        /**
         * Evaluate the Divergent component of motion.
         */
        void evaluateDCM();

        /**
         * Evaluate the CoM position and velocity.
         */
        void evaluateCoM();

    public:

        /**
         * Initialize the walking FK solver.
         * @param config config of the FK solver;
         * @param model iDynTree model.
         * @return true on success, false otherwise
         */
        bool initialize(const yarp::os::Searchable& config,
                        const iDynTree::Model& model);

        /**
         * Evaluate the world to base transformation
         * @param rootTransform transformation from the world to the root frame.
         * @param rootTwist twist of the root frame.
         * @return true/false in case of success/failure.
         */
        void evaluateWorldToBaseTransformation(const iDynTree::Transform& rootTransform,
                                               const iDynTree::Twist& rootTwist);

        /**
         * Evaluate the world to base transformation
         * @note: During the walking task the frame shift from the left to the right foot.
         * The base frame is attached where the foot should be (information sent by the planner)
         * @param leftFootTransform transformation from the world to the left foot frame (l_sole);
         * @param rightFootTransform transformation from the world to the right foot frame (r_sole);
         * @param isLeftFixedFrame true if the main frame of the left foot is fixed one.
         * @return true/false in case of success/failure.
         */
        bool evaluateWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform,
                                               const iDynTree::Transform& rightFootTransform,
                                               const bool& isLeftFixedFrame);

        /**
         * Set the base for the onTheFly feature
         * @return true/false in case of success/failure.
         */
        bool setBaseOnTheFly();

        /**
         * Set the internal state of the robot (joint position and velocity)
         * @param positionFeedbackInRadians joint position feedback expressed in radians;
         * @param velocityFeedbackInRadians joint velocity feedback expressed in radians per seconds.
         */
        bool setInternalRobotState(const iDynTree::VectorDynSize& positionFeedbackInRadians,
                                   const iDynTree::VectorDynSize& velocityFeedbackInRadians);

        /**
         * Get the CoM position.
         * @return CoM position
         */
        const iDynTree::Position& getCoMPosition();

        /**
         * Get the CoM velocity.
         * @return CoM velocity
         */
        const iDynTree::Vector3& getCoMVelocity();

        /**
         * Get the 2d-Divergent component of motion.
         * @return the 2d-Divergent component of motion
         */
        const iDynTree::Vector2& getDCM();

        /**
         * Return the transformation between the left foot frame (l_sole) and the world reference frame.
         * @return world_H_left_frame.
         */
        iDynTree::Transform getLeftFootToWorldTransform();

        /**
         * Return the transformation between the right foot frame (r_sole) and the world reference frame.
         * @return world_H_right_frame.
         */
        iDynTree::Transform getRightFootToWorldTransform();

        /**
         * Return the transformation between the left hand frame and the world reference frame.
         * @return world_H_left_hand.
         */
        iDynTree::Transform getLeftHandToWorldTransform();

        /**
         * Return the transformation between the right hand frame and the world reference frame.
         * @return world_H_right_hand.
         */
        iDynTree::Transform getRightHandToWorldTransform();

        /**
         * Return the transformation between the head frame and the world reference frame.
         * @return world_H_head.
         */
        iDynTree::Transform getHeadToWorldTransform();

        /**
         * Return the transformation between the root frame and the world reference frame.
         * @return world_H_root_frame.
         */
        iDynTree::Transform getRootLinkToWorldTransform();

        /**
         * Return the root link velocity.
         * @return the root link velocity expressed with the mixed representation.
         */
        iDynTree::Twist getRootLinkVelocity();

        /**
         * Return the neck orientation.
         * @return the rotation matrix between the neck and the reference frame.
         */
        iDynTree::Rotation getNeckOrientation();

        /**
         * Get the left foot jacobian.
         * @oaram jacobian is the left foot jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getLeftFootJacobian(iDynTree::MatrixDynSize &jacobian);

        /**
         * Get the right foot jacobian.
         * @oaram jacobian is the right foot jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getRightFootJacobian(iDynTree::MatrixDynSize &jacobian);

        /**
         * Get the left hand jacobian.
         * @oaram jacobian is the left hand jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getLeftHandJacobian(iDynTree::MatrixDynSize &jacobian);

        /**
         * Get the right hand jacobian.
         * @oaram jacobian is the right hand jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getRightHandJacobian(iDynTree::MatrixDynSize &jacobian);

        /**
         * Get the neck jacobian.
         * @oaram jacobian is the neck jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getNeckJacobian(iDynTree::MatrixDynSize &jacobian);

        /**
         * Get the CoM jacobian.
         * @oaram jacobian is the CoM jacobian matrix
         * @return true/false in case of success/failure.
         */
        bool getCoMJacobian(iDynTree::MatrixDynSize &jacobian);

        bool getRootLinkJacobian(iDynTree::MatrixDynSize &jacobian);
        /**
         * Get the joint position
         * @return the joint position expressed in radians
         */
        const iDynTree::VectorDynSize& getJointPos();
    };
};
#endif
