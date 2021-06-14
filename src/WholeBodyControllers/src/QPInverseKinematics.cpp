/**
 * @file WalkingQPInverseKinematics.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>, Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// Workaround for Eigen 3.3 on MSVC
#ifdef WALKING_CONTROLLERS_EIGEN_3_3_WORKAROUND
#include <eigen_workaround/SparseCwiseUnaryOp.h>
#endif

// std
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>

#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics.h>

using namespace WalkingControllers;

bool WalkingQPIK::initialize(const yarp::os::Searchable &config,
                             const int &actuatedDOFs,
                             const iDynTree::VectorDynSize& maxJointsVelocity,
                             const iDynTree::VectorDynSize& maxJointsPosition,
                             const iDynTree::VectorDynSize& minJointsPosition)
{
    m_actuatedDOFs = actuatedDOFs;
    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for QP-IK solver.";
        return false;
    }

    m_useCoMAsConstraint = config.check("use_com_as_constraint", yarp::os::Value(false)).asBool();
    bool enableHandRetargeting = config.check("use_hand_retargeting", yarp::os::Value(false)).asBool();
    bool enableJointRetargeting = config.check("use_joint_retargeting", yarp::os::Value(false)).asBool();

    if(enableHandRetargeting && enableJointRetargeting)
    {
        yError() << "[initialize] You cannot enable both hand and joint retargeting.";
        return false;
    }

    if(enableJointRetargeting)
        m_retargetingType = RetargetingType::jointRetargeting;
    else if(enableHandRetargeting)
        m_retargetingType = RetargetingType::handRetargeting;
    else
        m_retargetingType = RetargetingType::none;

    m_useJointsLimitsConstraint = config.check("use_joint_limits_constraint", yarp::os::Value(false)).asBool();

    // TODO in the future the number of constraints should be added inside
    // the configuration file
    // set the number of variables and the number of constraints
    // the number of variables is equal to the number of joints plus
    // 6 (position + attitude) degree of freedom related to the base.
    m_numberOfVariables = m_actuatedDOFs + 6;


    // the number of constraints is equal to the number of joints plus
    // 12 (position + attitude) of the left and right feet
    setNumberOfConstraints();

    // resize vectors
    m_gradient.resize(m_numberOfVariables);
    m_gradient.zero();
    m_lowerBound.resize(m_numberOfConstraints);
    m_lowerBound.zero();
    m_upperBound.resize(m_numberOfConstraints);
    m_upperBound.zero();
    m_solution.resize(m_numberOfVariables);
    m_desiredJointVelocitiesOutput.resize(m_actuatedDOFs);

    m_hessianDense.resize(m_numberOfVariables, m_numberOfVariables);
    m_constraintsMatrixSparse.resize(m_numberOfConstraints, m_numberOfVariables);

    // resize Jacobians matrices
    m_comJacobian.resize(3, m_numberOfVariables);
    m_neckJacobian.resize(3, m_numberOfVariables);
    m_leftFootJacobian.resize(6, m_numberOfVariables);
    m_rightFootJacobian.resize(6, m_numberOfVariables);
    m_leftHandJacobian.resize(6, m_numberOfVariables);
    m_rightHandJacobian.resize(6, m_numberOfVariables);

    m_regularizationTerm.resize(m_actuatedDOFs);
    m_retargetingJointValue.resize(m_actuatedDOFs);
    m_jointPosition.resize(m_actuatedDOFs);

    // preprare constant matrix necessary for the QP problem
    if(!initializeMatrices(config))
    {
        yError() << "[initialize] Unable to Initialize the constant matrix.";
        return false;
    }

    initializeSolverSpecificMatrices();

    if(m_retargetingType == RetargetingType::handRetargeting)
        if(!initializeHandRetargeting(config))
        {
            yError() << "[initialize] Unable to Initialize the hand retargeting.";
            return false;
        }

    if(m_retargetingType == RetargetingType::jointRetargeting)
        if(!initializeJointRetargeting(config))
        {
            yError() << "[initialize] Unable to Initialize the joint retargeting.";
            return false;
        }

    if(!setJointsBounds(maxJointsVelocity, maxJointsPosition, minJointsPosition))
    {
        yError() << "[initialize] Unable to set the joint bounds.";
        return false;
    }

    instantiateSolver();

    return true;
}

bool WalkingQPIK::setJointsBounds(const iDynTree::VectorDynSize& jointVelocitiesBounds,
                                  const iDynTree::VectorDynSize& jointPositionsUpperBounds,
                                  const iDynTree::VectorDynSize& jointPositionsLowerBounds)
{
    if(jointVelocitiesBounds.size() != jointPositionsUpperBounds.size() ||
       jointPositionsLowerBounds.size() != jointPositionsUpperBounds.size())
    {
        yError() << "[setJointsBounds] The size of the vector limits has to be equal.";
        return false;
    }
    if(jointVelocitiesBounds.size() != m_actuatedDOFs)
    {
        yError() << "[setJointsBounds] The size of the vector limits has to be equal to ."
                 << "the number of the joint";
        return false;
    }
    m_jointVelocitiesBounds = jointVelocitiesBounds;
    m_jointPositionsUpperBounds = jointPositionsUpperBounds;
    m_jointPositionsLowerBounds = jointPositionsLowerBounds;
    return true;
}

bool WalkingQPIK::initializeMatrices(const yarp::os::Searchable& config)
{
    // evaluate constant sub-matrix of the hessian matrix
    // get the CoM weight
    if(!m_useCoMAsConstraint)
    {
        if(!YarpUtilities::getVectorFromSearchable(config, "com_weight", m_comWeight))
        {
            yError() << "Initialization failed while reading com_weight vector.";
            return false;
        }
    }

    m_jointRegularizationGains.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_regularization_gains",
                                               m_jointRegularizationGains))
    {
        yError() << "Initialization failed while reading jointRegularizationGains vector.";
        return false;
    }

     // if the joint retargeting is enabled the weights of the cost function are time-variant.
    if (m_retargetingType != RetargetingType::jointRetargeting)
    {
        if(!YarpUtilities::getNumberFromSearchable(config, "neck_weight", m_neckWeight))
        {
            yError() << "Initialization failed while reading neck_weight vector.";
            return false;
        }

        // set the matrix related to the joint regularization
        m_jointRegularizationWeights.resize(m_actuatedDOFs);
        if(!YarpUtilities::getVectorFromSearchable(config, "joint_regularization_weights", m_jointRegularizationWeights))
        {
            yError() << "Initialization failed while reading jointRegularizationWeights vector.";
            return false;
        }

        // if the joint retargeting is NOT enabled both the jointRegularizationWeights and the
        // jointRegularizationGains are constant. For this reason we can compute their product
        m_jointRegularizationGainsTimeWeights.resize(m_actuatedDOFs);
        iDynTree::toEigen(m_jointRegularizationGainsTimeWeights) = iDynTree::toEigen(m_jointRegularizationWeights).cwiseProduct(iDynTree::toEigen(m_jointRegularizationGains));
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_posFoot", m_kPosFoot))
    {
        yError() << "Initialization failed while reading k_posFoot.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_attFoot", m_kAttFoot))
    {
        yError() << "Initialization failed while reading k_attFoot.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_neck", m_kNeck))
    {
        yError() << "Initialization failed while reading k_neck.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_posCom", m_kCom))
    {
        yError() << "Initialization failed while reading k_posCom.";
        return false;
    }

    // get the regularization term
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_regularization", m_regularizationTerm))
    {
        yError() << "[initialize] Unable to get the iDynTreeVector from searchable.";
        return false;
    }

    iDynTree::toEigen(m_regularizationTerm) = iDynTree::toEigen(m_regularizationTerm) *
        iDynTree::deg2rad(1);

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[initialize] Unable to set the additional rotation.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_joint_limit_lower_bound", m_kJointLimitsLowerBound))
    {
        yError() << "Initialization failed while reading a double.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_joint_limit_upper_bound", m_kJointLimitsUpperBound))
    {
        yError() << "Initialization failed while reading a double.";
        return false;
    }

    return true;
}

bool WalkingQPIK::initializeHandRetargeting(const yarp::os::Searchable& config)
{
    if(!YarpUtilities::getNumberFromSearchable(config, "k_posHand", m_kPosHand))
    {
        yError() << "Initialization failed while reading k_posHand.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "k_attHand", m_kAttHand))
    {
        yError() << "Initialization failed while reading k_attHand.";
        return false;
    }

    m_handWeightWalkingVector.resize(6);
    if(!YarpUtilities::getVectorFromSearchable(config, "hand_weight_walking", m_handWeightWalkingVector))
    {
        yError() << "Initialization failed while reading the yarp vector.";
        return false;
    }

    m_handWeightStanceVector.resize(6);
    if(!YarpUtilities::getVectorFromSearchable(config, "hand_weight_stance", m_handWeightStanceVector))
    {
        yError() << "Initialization failed while reading the yarp vector.";
        return false;
    }

    double smoothingTime;
    if(!YarpUtilities::getNumberFromSearchable(config, "smoothing_time", smoothingTime))
    {
        yError() << "Initialization failed while reading smoothingTime.";
        return false;
    }

    double dT;
    if(!YarpUtilities::getNumberFromSearchable(config, "sampling_time", dT))
    {
        yError() << "Initialization failed while a double.";
        return false;
    }


    m_handWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, dT, smoothingTime);
    m_handWeightSmoother->init(m_handWeightStanceVector);

    return true;
}

bool WalkingQPIK::initializeJointRetargeting(const yarp::os::Searchable& config)
{
    double smoothingTime;
    if(!YarpUtilities::getNumberFromSearchable(config, "smoothing_time", smoothingTime))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading smoothingTime.";
        return false;
    }

    double dT;
    if(!YarpUtilities::getNumberFromSearchable(config, "sampling_time", dT))
    {
        yError() << "Initialization failed while a double.";
        return false;
    }

    // joint retargeting
    m_jointRetargetingGains.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_retargeting_gains", m_jointRetargetingGains))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading jointRetargetingGains vector.";
        return false;
    }

    // if the joint retargeting is enable the weights of the cost function are time variant
    m_jointRetargetingWeightWalking.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_retargeting_weight_walking", m_jointRetargetingWeightWalking))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the yarp vector.";
        return false;
    }

    m_jointRetargetingWeightStance.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_retargeting_weight_stance", m_jointRetargetingWeightStance))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the yarp vector.";
        return false;
    }

    m_jointRetargetingWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, dT, smoothingTime);
    m_jointRetargetingWeightSmoother->init(m_jointRetargetingWeightStance);

    // joint regularization
    m_jointRegularizationWeightWalking.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_regularization_weight_walking", m_jointRegularizationWeightWalking))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the yarp vector.";
        return false;
    }

    m_jointRegularizationWeightStance.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorFromSearchable(config, "joint_regularization_weight_stance", m_jointRegularizationWeightStance))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the yarp vector.";
        return false;
    }

    m_jointRegularizationWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(m_actuatedDOFs, dT, smoothingTime);
    m_jointRegularizationWeightSmoother->init(m_jointRegularizationWeightStance);

    // torso
    if(!YarpUtilities::getNumberFromSearchable(config, "torso_weight_walking", m_torsoWeightWalking))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the double.";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "torso_weight_stance", m_torsoWeightStance))
    {
        yError() << "[initializeJointRetargeting] Initialization failed while reading the double.";
        return false;
    }

    m_torsoWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, dT, smoothingTime);
    m_torsoWeightSmoother->init(yarp::sig::Vector(1, m_torsoWeightStance));

    return true;
}

bool WalkingQPIK::setRobotState(WalkingFK& kinDynWrapper)
{

    m_leftHandToWorldTransform = kinDynWrapper.getLeftHandToWorldTransform();
    m_rightHandToWorldTransform = kinDynWrapper.getRightHandToWorldTransform();

    m_jointPosition = kinDynWrapper.getJointPos();
    m_leftFootToWorldTransform = kinDynWrapper.getLeftFootToWorldTransform();
    m_rightFootToWorldTransform = kinDynWrapper.getRightFootToWorldTransform();
    m_neckOrientation = kinDynWrapper.getNeckOrientation();
    m_comPosition = kinDynWrapper.getCoMPosition();
    m_comPosition(2) = kinDynWrapper.getRootLinkToWorldTransform().getPosition()(2);

    return true;
}

void WalkingQPIK::setPhase(const bool& isStancePhase)
{
    if(isStancePhase)
    {
        if(m_retargetingType == RetargetingType::handRetargeting)
            m_handWeightSmoother->computeNextValues(m_handWeightStanceVector);

        else if(m_retargetingType == RetargetingType::jointRetargeting)
        {
            m_torsoWeightSmoother->computeNextValues(yarp::sig::Vector(1, m_torsoWeightStance));
            m_jointRetargetingWeightSmoother->computeNextValues(m_jointRetargetingWeightStance);
            m_jointRegularizationWeightSmoother->computeNextValues(m_jointRegularizationWeightStance);
        }
    }
    else
    {
        if(m_retargetingType == RetargetingType::handRetargeting)
            m_handWeightSmoother->computeNextValues(m_handWeightWalkingVector);

        else if(m_retargetingType == RetargetingType::jointRetargeting)
        {
            m_torsoWeightSmoother->computeNextValues(yarp::sig::Vector(1, m_torsoWeightWalking));
            m_jointRetargetingWeightSmoother->computeNextValues(m_jointRetargetingWeightWalking);
            m_jointRegularizationWeightSmoother->computeNextValues(m_jointRegularizationWeightWalking);
        }
    }
}

void WalkingQPIK::setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation)
{
    m_desiredNeckOrientation =  desiredNeckOrientation * m_additionalRotation;
}

bool WalkingQPIK::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
{
    if(comJacobian.rows() != 3)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 3.";
        return false;
    }
    if(comJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }
    m_comJacobian = comJacobian;

    return true;
}

bool WalkingQPIK::setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian)
{
    if(leftFootJacobian.rows() != 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(leftFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_leftFootJacobian = leftFootJacobian;

    return true;
}

bool WalkingQPIK::setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian)
{
    if(rightFootJacobian.rows() != 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_rightFootJacobian = rightFootJacobian;

    return true;
}

bool WalkingQPIK::setLeftHandJacobian(const iDynTree::MatrixDynSize& leftHandJacobian)
{
    if(m_retargetingType != RetargetingType::handRetargeting)
        return true;

    if(leftHandJacobian.rows() != 6)
    {
        yError() << "[setLeftHandJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(leftHandJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setLeftHandJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_leftHandJacobian = leftHandJacobian;

    return true;
}

bool WalkingQPIK::setRightHandJacobian(const iDynTree::MatrixDynSize& rightHandJacobian)
{
    if(m_retargetingType != RetargetingType::handRetargeting)
        return true;

    if(rightHandJacobian.rows() != 6)
    {
        yError() << "[setRightHandJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightHandJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setRightHandJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_rightHandJacobian = rightHandJacobian;

    return true;
}

bool WalkingQPIK::setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian)
{
    if(neckJacobian.rows() != 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(neckJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_neckJacobian) = iDynTree::toEigen(neckJacobian).block(3, 0, 3,
                                                                              m_actuatedDOFs + 6);

    return true;
}

bool WalkingQPIK::setDesiredJointPosition(const iDynTree::VectorDynSize& regularizationTerm)
{
    if(regularizationTerm.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointPosition] The number of the desired joint position has to be "
                 << "equal to the number of actuated joints";
        return false;
    }

    m_regularizationTerm = regularizationTerm;

    return true;
}

void WalkingQPIK::setDesiredFeetTransformation(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                               const iDynTree::Transform& desiredRightFootToWorldTransform)
{
    m_desiredLeftFootToWorldTransform = desiredLeftFootToWorldTransform;
    m_desiredRightFootToWorldTransform = desiredRightFootToWorldTransform;
}

void WalkingQPIK::setDesiredFeetTwist(const iDynTree::Twist& leftFootTwist,
                                      const iDynTree::Twist& rightFootTwist)
{
    m_desiredLeftFootTwist = leftFootTwist;
    m_desiredRightFootTwist = rightFootTwist;
}

void WalkingQPIK::setDesiredHandsTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform,
                                                const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    if(m_retargetingType != RetargetingType::handRetargeting)
        return;

    m_desiredLeftHandToWorldTransform = desiredLeftHandToWorldTransform;
    m_desiredRightHandToWorldTransform = desiredRightHandToWorldTransform;
}

bool WalkingQPIK::setDesiredRetargetingJoint(const iDynTree::VectorDynSize& jointPosition)
{
    // if the joint retargeting is not used return
    if(m_retargetingType != RetargetingType::jointRetargeting)
        return true;

    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[WalkingQPIK::setDesiredRetargetingJoint] The size of the jointPosition vector is not coherent with the "
                 << "number of the actuated Joint";
        return false;
    }

    m_retargetingJointValue = jointPosition;

    return true;
}

void WalkingQPIK::setDesiredHandsTwist(const iDynTree::Twist& leftHandTwist,
                                       const iDynTree::Twist& rightHandTwist)
{
    m_desiredLeftHandTwist = leftHandTwist;
    m_desiredRightHandTwist = rightHandTwist;
}

void WalkingQPIK::setDesiredCoMVelocity(const iDynTree::Vector3& comVelocity)
{
    m_desiredComVelocity = comVelocity;
}

void WalkingQPIK::setDesiredCoMPosition(const iDynTree::Position& desiredComPosition)
{
    m_desiredComPosition = desiredComPosition;
}

void WalkingQPIK::evaluateHessianMatrix()
{
    // in that case the hessian matrix is related only to neck orientation and to
    // the joint angle
    auto hessianDense(iDynTree::toEigen(m_hessianDense));

    // if the joint retargeting is enable the weights of the cost function are time variant
    if (m_retargetingType != RetargetingType::jointRetargeting)
    {
        hessianDense = iDynTree::toEigen(m_neckJacobian).transpose() * m_neckWeight * iDynTree::toEigen(m_neckJacobian);
        hessianDense.bottomRightCorner(m_actuatedDOFs, m_actuatedDOFs) += iDynTree::toEigen(m_jointRegularizationWeights).asDiagonal();
    }
    else
    {
        hessianDense = iDynTree::toEigen(m_neckJacobian).transpose() *
            m_torsoWeightSmoother->getPos()(0) *
            iDynTree::toEigen(m_neckJacobian);
        hessianDense.bottomRightCorner(m_actuatedDOFs, m_actuatedDOFs) +=
            (iDynTree::toEigen(m_jointRegularizationWeightSmoother->getPos()) +
             iDynTree::toEigen(m_jointRetargetingWeightSmoother->getPos())).asDiagonal();
    }

    if(m_retargetingType == RetargetingType::handRetargeting)
    {
        // think about the possibility to project in the null space the joint regularization
        hessianDense +=  iDynTree::toEigen(m_leftHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal()
            * iDynTree::toEigen(m_leftHandJacobian)
            + iDynTree::toEigen(m_rightHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal()
            * iDynTree::toEigen(m_rightHandJacobian);
    }

    if(!m_useCoMAsConstraint)
    {
        hessianDense += iDynTree::toEigen(m_comJacobian).transpose() *
            iDynTree::toEigen(m_comWeight).asDiagonal() * iDynTree::toEigen(m_comJacobian);
    }
}

void WalkingQPIK::evaluateGradientVector()
{
    auto gradient(iDynTree::toEigen(m_gradient));

    auto neckJacobian(iDynTree::toEigen(m_neckJacobian));

    auto jointRegularizationGains(iDynTree::toEigen(m_jointRegularizationGains));
    auto jointPosition(iDynTree::toEigen(m_jointPosition));
    auto regularizationTerm(iDynTree::toEigen(m_regularizationTerm));

    auto comJacobian(iDynTree::toEigen(m_comJacobian));
    auto comWeight(iDynTree::toEigen(m_comWeight));
    auto comPosition(iDynTree::toEigen(m_comPosition));
    auto desiredComPosition(iDynTree::toEigen(m_desiredComPosition));
    auto desiredComVelocity(iDynTree::toEigen(m_desiredComVelocity));

    // Neck orientation
    iDynTree::Matrix3x3 errorNeckAttitude = iDynTreeUtilities::Rotation::skewSymmetric(m_neckOrientation * m_desiredNeckOrientation.inverse());
    if(m_retargetingType != RetargetingType::jointRetargeting)
    {
        auto jointRegularizationGainsTimeWeights(iDynTree::toEigen(m_jointRegularizationGainsTimeWeights));

        gradient = -neckJacobian.transpose() * m_neckWeight * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)));

        // g = Weight * K_p * (regularizationTerm - jointPosition)
        // Weight  and K_p are two diagonal matrices so their product can be also evaluated multiplying component-wise
        // the elements of the vectors and then generating the diagonal matrix
        gradient.tail(m_actuatedDOFs) -= jointRegularizationGainsTimeWeights.asDiagonal() * (regularizationTerm - jointPosition);
    }
    else
    {
        auto jointRetargetingGains(iDynTree::toEigen(m_jointRetargetingGains));
        auto jointRetargetingValues(iDynTree::toEigen(m_retargetingJointValue));

        gradient = -neckJacobian.transpose() * m_torsoWeightSmoother->getPos()(0)
            * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)));

        // g = Weight * K_p * (regularizationTerm - jointPosition)
        // Weight  and K_p are two diagonal matrices so their product can be also evaluated multiplying component-wise
        // the elements of the vectors and then generating the diagonal matrix
        const auto& jointRegularizationWeight = m_jointRegularizationWeightSmoother->getPos();
        gradient.tail(m_actuatedDOFs) += (-jointRegularizationGains.cwiseProduct(iDynTree::toEigen(jointRegularizationWeight))).asDiagonal()
            * (regularizationTerm - jointPosition);

        const auto& jointRetargetingWeight = m_jointRetargetingWeightSmoother->getPos();
        gradient.tail(m_actuatedDOFs) += (-jointRetargetingGains.cwiseProduct(iDynTree::toEigen(jointRetargetingWeight))).asDiagonal()
            * (jointRetargetingValues - jointPosition);
    }

    if(m_retargetingType == RetargetingType::handRetargeting)
    {
        Eigen::Map<Eigen::Vector3d> leftHandCorrectionLinearVel(iDynTree::toEigen(m_leftHandCorrection.getLinearVec3()));
        Eigen::Map<Eigen::Vector3d> leftHandCorrectionAngularVel(iDynTree::toEigen(m_leftHandCorrection.getAngularVec3()));
        Eigen::Map<Eigen::Vector3d> rightHandCorrectionLinearVel(iDynTree::toEigen(m_rightHandCorrection.getLinearVec3()));
        Eigen::Map<Eigen::Vector3d> rightHandCorrectionAngularVel(iDynTree::toEigen(m_rightHandCorrection.getAngularVec3()));

        auto saturationLambda = [](const Eigen::Map<Eigen::Vector3d>& input, double saturationValue) -> Eigen::Vector3d {
            if (saturationValue > 0)
            {
                if (saturationValue < 1e-10)
                {
                    Eigen::Vector3d nullVector;
                    nullVector.setZero();

                    return nullVector;
                }
                else
                {
                    Eigen::Vector3d result;
                    result = input / std::max(1.0, input.norm()/saturationValue); //Rescale the vector to limit it to saturationValue
                    return result;
                }
            }

            return input;
        };

        //  left hand
        leftHandCorrectionLinearVel = m_kPosHand * iDynTree::toEigen(m_leftHandToWorldTransform.getPosition()
                                                                  - m_desiredLeftHandToWorldTransform.getPosition());
        leftHandCorrectionLinearVel = saturationLambda(leftHandCorrectionLinearVel, m_maxHandLinearVelocity);

        iDynTree::Matrix3x3 leftHandAttitudeError = iDynTreeUtilities::Rotation::skewSymmetric(m_leftHandToWorldTransform.getRotation() *
                                                                                            m_desiredLeftHandToWorldTransform.getRotation().inverse());

        leftHandCorrectionAngularVel = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(leftHandAttitudeError)));
        leftHandCorrectionAngularVel = saturationLambda(leftHandCorrectionAngularVel, m_maxHandAngularVelocity);


        //  right hand
        rightHandCorrectionLinearVel = m_kPosHand * iDynTree::toEigen(m_rightHandToWorldTransform.getPosition()
                                                                   - m_desiredRightHandToWorldTransform.getPosition());
        rightHandCorrectionLinearVel = saturationLambda(rightHandCorrectionLinearVel, m_maxHandLinearVelocity);


        iDynTree::Matrix3x3 rightHandAttitudeError = iDynTreeUtilities::Rotation::skewSymmetric(m_rightHandToWorldTransform.getRotation() *
                                                                                             m_desiredRightHandToWorldTransform.getRotation().inverse());

        rightHandCorrectionAngularVel = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(rightHandAttitudeError)));
        rightHandCorrectionAngularVel = saturationLambda(rightHandCorrectionAngularVel, m_maxHandAngularVelocity);


        gradient += - iDynTree::toEigen(m_leftHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal() * (-iDynTree::toEigen(m_leftHandCorrection))
            - iDynTree::toEigen(m_rightHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal() * (-iDynTree::toEigen(m_rightHandCorrection));
    }

    if(!m_useCoMAsConstraint)
    {
        gradient += -comJacobian.transpose() * comWeight.asDiagonal() *
            (desiredComVelocity - m_kCom * (comPosition - desiredComPosition));
    }
}

void WalkingQPIK::evaluateLinearConstraintMatrix()
{
    copyDenseIntoSparse(m_leftFootJacobian, 0, 0, m_constraintsMatrixSparse);
    copyDenseIntoSparse(m_rightFootJacobian, 6, 0, m_constraintsMatrixSparse);

    if(m_useCoMAsConstraint)
        copyDenseIntoSparse(m_comJacobian, 6 + 6, 0, m_constraintsMatrixSparse);
}

void WalkingQPIK::evaluateBounds()
{
    auto lowerBound(iDynTree::toEigen(m_lowerBound));
    auto upperBound(iDynTree::toEigen(m_upperBound));
    auto leftFootCorrectionLinear(iDynTree::toEigen(m_leftFootCorrection.getLinearVec3()));
    auto leftFootCorrectionAngular(iDynTree::toEigen(m_leftFootCorrection.getAngularVec3()));
    auto rightFootCorrectionLinear(iDynTree::toEigen(m_rightFootCorrection.getLinearVec3()));
    auto rightFootCorrectionAngular(iDynTree::toEigen(m_rightFootCorrection.getAngularVec3()));

    // left foot
    leftFootCorrectionLinear = m_kPosFoot * iDynTree::toEigen((m_leftFootToWorldTransform.getPosition() -
                                                               m_desiredLeftFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorLeftAttitude = iDynTreeUtilities::Rotation::skewSymmetric(m_leftFootToWorldTransform.getRotation() *
                                                                                    m_desiredLeftFootToWorldTransform.getRotation().inverse());

    leftFootCorrectionAngular = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorLeftAttitude)));

    // right foot
    rightFootCorrectionLinear = m_kPosFoot * iDynTree::toEigen((m_rightFootToWorldTransform.getPosition() -
                                                                         m_desiredRightFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorRightAttitude = iDynTreeUtilities::Rotation::skewSymmetric(m_rightFootToWorldTransform.getRotation() *
                                                                                     m_desiredRightFootToWorldTransform.getRotation().inverse());

    rightFootCorrectionAngular = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorRightAttitude)));

    if((m_desiredLeftFootTwist(0) == m_desiredLeftFootTwist(1)) && (m_desiredLeftFootTwist(0) == 0))
    {
        lowerBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist);
        upperBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist);
    }
    else
    {
        lowerBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist) - iDynTree::toEigen(m_leftFootCorrection);
        upperBound.segment<6>(0) = lowerBound.segment<6>(0);
    }

    if((m_desiredRightFootTwist(0) == m_desiredRightFootTwist(1)) && (m_desiredRightFootTwist(0) == 0))
    {
        lowerBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist);
        upperBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist);
    }
    else
    {
        lowerBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist) - iDynTree::toEigen(m_rightFootCorrection);
        upperBound.segment<6>(6) = lowerBound.segment<6>(6);
    }
    if(m_useCoMAsConstraint)
    {
        lowerBound.segment<2>(12) = iDynTree::toEigen(m_desiredComVelocity).head<2>()
            - m_kCom * (iDynTree::toEigen(m_comPosition).head<2>() -  iDynTree::toEigen(m_desiredComPosition).head<2>());

        lowerBound.segment<1>(14) = iDynTree::toEigen(m_desiredComVelocity).tail<1>()
            - 0.5 * (iDynTree::toEigen(m_comPosition).tail<1>() -  iDynTree::toEigen(m_desiredComPosition).tail<1>());
        upperBound.segment<3>(12) = lowerBound.segment<3>(12);
    }

    setJointVelocitiesBounds();
}

const iDynTree::VectorDynSize& WalkingQPIK::getSolution() const
{
    return m_solution;
}

const iDynTree::VectorDynSize& WalkingQPIK::getDesiredJointVelocities() const
{
    return m_desiredJointVelocitiesOutput;
}

iDynTree::VectorDynSize WalkingQPIK::getLeftFootError()
{
    iDynTree::VectorDynSize output(6);
    iDynTree::toEigen(output) = iDynTree::toEigen(m_lowerBound).block(0, 0, 6, 1)
        - iDynTree::toEigen(m_leftFootJacobian) * iDynTree::toEigen(m_solution);
    return output;
}

iDynTree::VectorDynSize WalkingQPIK::getRightFootError()
{
    iDynTree::VectorDynSize output(6);
    iDynTree::toEigen(output) = iDynTree::toEigen(m_lowerBound).block(6, 0, 6, 1)
        - iDynTree::toEigen(m_rightFootJacobian) * iDynTree::toEigen(m_solution);
    return output;
}

const iDynTree::MatrixDynSize& WalkingQPIK::getHessianMatrix() const
{
    return m_hessianDense;
}

const iDynSparseMatrix& WalkingQPIK::getConstraintMatrix() const
{
    return m_constraintsMatrixSparse;
}

const iDynTree::VectorDynSize& WalkingQPIK::getUpperBound() const
{
    return m_upperBound;
}

const iDynTree::VectorDynSize& WalkingQPIK::getLowerBound() const
{
    return m_lowerBound;
}

const iDynTree::VectorDynSize& WalkingQPIK::getGradient() const
{
    return m_gradient;
}
