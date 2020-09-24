#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <WalkingControllers/RobotInterface/Helper.h>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

bool RobotInterface::getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                   std::pair<int, double>& worstError)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotInterface::getWorstError] The encoder I/F is not ready";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackDeg.data()))
    {
        yError() << "[RobotInterface::getWorstError] Error reading encoders.";
        return false;
    }

    // clear the std::pair
    worstError.first = 0;
    worstError.second = 0.0;
    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        if (m_currentJointInteractionMode[i] == yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF
            && m_isGoodTrackingRequired[i])
        {
            currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
            absoluteJointErrorRad = std::abs(iDynTreeUtilities::shortestAngularDistance(currentJointPositionRad,
                                                                                        desiredJointPositionsRad(i)));
            if(absoluteJointErrorRad > worstError.second)
            {
                worstError.first = i;
                worstError.second = absoluteJointErrorRad;
            }
        }
    }
    return true;
}

bool RobotInterface::getFeedbacksRaw(unsigned int maxAttempts)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotInterface::getFeedbacksRaw] Encoders I/F is not ready";
        return false;
    }

    bool okPosition = false;
    bool okVelocity = false;

    bool okLeftWrench = false;
    bool okRightWrench = false;

    bool okBaseEstimation = !m_useExternalRobotBase;

    unsigned int attempt = 0;
    do
    {
        if(!okPosition)
            okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());

        if(!okVelocity)
            okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okLeftWrench)
        {
            yarp::sig::Vector *leftWrenchRaw = NULL;
            leftWrenchRaw = m_leftWrenchPort.read(false);
            if(leftWrenchRaw != NULL)
            {
                m_leftWrenchInput = *leftWrenchRaw;
                okLeftWrench = true;
            }
        }

        if(!okRightWrench)
        {
            yarp::sig::Vector *rightWrenchRaw = NULL;
            rightWrenchRaw = m_rightWrenchPort.read(false);
            if(rightWrenchRaw != NULL)
            {
                m_rightWrenchInput = *rightWrenchRaw;
                okRightWrench = true;
            }
        }

        if(!okBaseEstimation)
        {
            yarp::sig::Vector *base = NULL;
            base = m_robotBasePort.read(false);
            if(base != NULL)
            {
                m_robotBaseTransform.setPosition(iDynTree::Position((*base)(0),
                                                                    (*base)(1),
                                                                    (*base)(2) - m_heightOffset));

                m_robotBaseTransform.setRotation(iDynTree::Rotation::RPY((*base)(3),
                                                                         (*base)(4),
                                                                         (*base)(5)));

                m_robotBaseTwist.setLinearVec3(iDynTree::Vector3(base->data() + 6, 3));
                m_robotBaseTwist.setAngularVec3(iDynTree::Vector3(base->data() + 6 + 3, 3));
                okBaseEstimation = true;
            }
        }

        if(okPosition && okVelocity && okLeftWrench && okRightWrench && okBaseEstimation)
        {
            for(unsigned j = 0 ; j < m_actuatedDOFs; j++)
            {
                m_positionFeedbackRad(j) = iDynTree::deg2rad(m_positionFeedbackDeg(j));
                m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDeg(j));
            }

            if(!iDynTree::toiDynTree(m_leftWrenchInput, m_leftWrench))
            {
                yError() << "[RobotInterface::getFeedbacksRaw] Unable to convert left foot wrench.";
                return false;
            }
            if(!iDynTree::toiDynTree(m_rightWrenchInput, m_rightWrench))
            {
                yError() << "[RobotInterface::getFeedbacksRaw] Unable to convert right foot wrench.";
                return false;
            }
            return true;
        }
        yarp::os::Time::delay(0.001);
        attempt++;
    } while (attempt < maxAttempts);

    yError() << "[RobotInterface::getFeedbacksRaw] The following readings failed:";
    if(!okPosition)
        yError() << "\t - Position encoders";

    if(!okVelocity)
        yError() << "\t - Velocity encoders";

    if(!okLeftWrench)
        yError() << "\t - Left wrench";

    if(!okRightWrench)
        yError() << "\t - Right wrench";

    if(!okBaseEstimation)
        yError() << "\t - Base estimation";

    return false;
}

bool RobotInterface::configureRobot(const yarp::os::Searchable& config)
{
    // robot name: used to connect to the robot
    std::string robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    double sampligTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    std::string name;
    if(!YarpUtilities::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotInterface::configureRobot] Unable to get the string from searchable.";
        return false;
    }

    yarp::os::Value *axesListYarp;
    if(!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotInterface::configureRobot] Unable to find joints_list into config file.";
        return false;
    }
    if(!YarpUtilities::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotInterface::configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value *iCubPartsYarp;
    if(!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[configureRobot] Unable to find remote_control_boards into config file.";
        return false;
    }
    if(!YarpUtilities::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    YarpUtilities::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    m_remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = m_remoteControlBoards.addList();
    for(auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", m_remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    // get the actuated DoFs
    m_actuatedDOFs = m_axesList.size();

    m_isGoodTrackingRequired.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorOfBooleanFromSearchable(config, "good_tracking_required",
                                                        m_isGoodTrackingRequired))
    {
        yError() << "[RobotInterface::configureRobot] Unable to find is_good_tracking_required into config file.";
        return false;
    }

    m_jointInteractionMode.resize(m_actuatedDOFs);
    m_currentJointInteractionMode.resize(m_actuatedDOFs);
    std::vector<bool> isJointInStiffMode(m_actuatedDOFs);
    m_stiffnessGainVector.resize(m_actuatedDOFs);
    m_dampingGainVector.resize(m_actuatedDOFs);
    if(!YarpUtilities::getVectorOfBooleanFromSearchable(config, "joint_is_stiff_mode",
                                                        isJointInStiffMode))
    {
        yError() << "[RobotInterface::configureRobot] Unable to find joint_is_stiff_mode into config file.";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config,"joint_stiffness_gain",m_stiffnessGainVector))
    {
        yError() << "[RobotInterface::configureRobot] Unable to find joint_stiffness_gain into config file.";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config,"joint_damping_gain",m_dampingGainVector))
    {
        yError() << "[RobotInterface::configureRobot] Unable to find joint_damping_gain into config file.";
        return false;
    }

    for (unsigned int i = 0; i < m_actuatedDOFs; i++)
    {
        if(isJointInStiffMode[i])
        {
            m_jointInteractionMode[i] = yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
        }
        else
        {
            m_jointInteractionMode[i] = yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT;
        }
    }

    for (unsigned int i = 0; i < m_actuatedDOFs; i++)
    {
        if(m_jointInteractionMode[i] == yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT
           && m_isGoodTrackingRequired[i])
        {
            yWarning() << "[configureRobot] The control mode of the the joint " << m_axesList[i]
                       << " is set to COMPLIANT. It is not possible to guarantee a good tracking.";
        }
    }

    // open the device
    if(!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    // obtain the interfaces
    if(!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[configureRobot] Cannot obtain IEncoders interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionControl interface";
        return false;
    }

    if(!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[configureRobot] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if(!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_limitsInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_interactionInterface) || !m_interactionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IInteractionMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_impedanceControlInterface) || !m_impedanceControlInterface)
    {
             yError() << "[configureRobot] Cannot obtain ImpedanceControl interface";
            return false;
    }

    // resize the buffers
    m_positionFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_velocityFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_positionFeedbackRad.resize(m_actuatedDOFs);
    m_velocityFeedbackRad.resize(m_actuatedDOFs);
    m_desiredJointPositionRad.resize(m_actuatedDOFs);
    m_desiredJointValueDeg.resize(m_actuatedDOFs);
    m_jointVelocitiesBounds.resize(m_actuatedDOFs);
    m_jointPositionsUpperBounds.resize(m_actuatedDOFs);
    m_jointPositionsLowerBounds.resize(m_actuatedDOFs);

    // m_positionFeedbackDegFiltered.resize(m_actuatedDOFs);
    // m_positionFeedbackDegFiltered.zero();

    m_velocityFeedbackDegFiltered.resize(m_actuatedDOFs);
    m_velocityFeedbackDegFiltered.zero();

    // check if the robot is alive
    bool okPosition = false;
    bool okVelocity = false;
    for (int i=0; i < 10 && !okPosition && !okVelocity; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());
        okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okPosition || !okVelocity)
            yarp::os::Time::delay(0.1);
    }
    if(!okPosition)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    if(!okVelocity)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    m_useVelocityFilter = config.check("use_joint_velocity_filter", yarp::os::Value("False")).asBool();
    if(m_useVelocityFilter)
    {
        double cutFrequency;
        if(!YarpUtilities::getNumberFromSearchable(config, "joint_velocity_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        // set filters
        // m_positionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(10, m_dT);
        m_velocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                 sampligTime);

        // m_positionFilter->init(m_positionFeedbackDeg);
        m_velocityFilter->init(m_velocityFeedbackDeg);
    }

    // get the limits
    double maxVelocity, minAngle, maxAngle, dummy;
    for(unsigned int i = 0; i < m_actuatedDOFs; i++)
    {
        if(!m_limitsInterface->getVelLimits(i, &dummy, &maxVelocity))
        {
            yError() << "[configure] Unable get the velocity limits of the joint: "
                     << m_axesList[i];
            return false;
        }

        m_jointVelocitiesBounds(i) = iDynTree::deg2rad(maxVelocity);


        if(!m_limitsInterface->getLimits(i, &minAngle, &maxAngle))
        {
            yError() << "[configure] Unable get the position limits of the joint: "
                     << m_axesList[i];
            return false;
        }

        m_jointPositionsUpperBounds(i) = iDynTree::deg2rad(maxAngle);
        m_jointPositionsLowerBounds(i) = iDynTree::deg2rad(minAngle);

    }

    m_useExternalRobotBase = config.check("use_external_robot_base", yarp::os::Value("False")).asBool();
    if(m_useExternalRobotBase)
    {
        m_robotBasePort.open("/" + name + "/robotBase:i");
        // connect port

        std::string floatingBasePortName;
        if(!YarpUtilities::getStringFromSearchable(config, "floating_base_port_name", floatingBasePortName))
        {
            yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(floatingBasePortName, "/" + name + "/robotBase:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/robotBase:i";
            return false;
        }
    }
    m_heightOffset = 0;


    // set the default control mode
    if(!m_interactionInterface->getInteractionModes(m_currentJointInteractionMode.data()))
    {
        yError() << "[RobotHelper::configure] Unable to get the interaction mode.";
        return  false;
    }
    if(!setInteractionMode(yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF))
    {
        yError() << "[RobotInterface::configureRobot] Unable to set the stiff control mode for all joints.";
        return false;
    }

    return true;
}

bool RobotInterface::configureForceTorqueSensors(const yarp::os::Searchable& config)
{
    std::string portInput, portOutput;

    // check if the config file is empty
    if(config.isNull())
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Empty configuration for the force torque sensors.";
        return false;
    }

    std::string name;
    if(!YarpUtilities::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }

    double sampligTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    // open and connect left foot wrench
    if(!YarpUtilities::getStringFromSearchable(config, "leftFootWrenchInputPort_name", portInput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to get "
            "the string from searchable.";
        return false;
    }
    if(!YarpUtilities::getStringFromSearchable(config, "leftFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_leftWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to connect to port "
                 << portOutput << " to " << "/" + name + portInput;
        return false;
    }

    // open and connect right foot wrench
    if(!YarpUtilities::getStringFromSearchable(config, "rightFootWrenchInputPort_name", portInput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpUtilities::getStringFromSearchable(config, "rightFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_rightWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "[RobotInterface::configureForceTorqueSensors] Unable to connect to port "
                 << portOutput << " to " << "/" + name + portInput;
        return false;
    }

    m_useWrenchFilter = config.check("use_wrench_filter", yarp::os::Value("False")).asBool();
    if(m_useWrenchFilter)
    {
        double cutFrequency;
        if(!YarpUtilities::getNumberFromSearchable(config, "wrench_cut_frequency", cutFrequency))
        {
            yError() << "[RobotInterface::configureForceTorqueSensors] Unable get double from searchable.";
            return false;
        }

        m_leftWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                   sampligTime);
        m_rightWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                    sampligTime);
    }
    return true;
}

bool RobotInterface::configurePIDHandler(const yarp::os::Bottle& config)
{
    m_PIDHandler = std::make_unique<WalkingPIDHandler>();
    return m_PIDHandler->initialize(config, m_robotDevice, m_remoteControlBoards);
}


bool RobotInterface::resetFilters()
{
    if(!getFeedbacksRaw(100))
    {
        yError() << "[RobotInterface::resetFilters] Unable to get the feedback from the robot";
        return false;
    }

    if(m_useVelocityFilter)
        m_velocityFilter->init(m_velocityFeedbackDeg);

    if(m_useWrenchFilter)
    {
        m_leftWrenchFilter->init(m_leftWrenchInput);
        m_rightWrenchFilter->init(m_rightWrenchInput);
    }

    return true;
}

bool RobotInterface::getFeedbacks(unsigned int maxAttempts)
{
    if(!getFeedbacksRaw(maxAttempts))
    {
        yError() << "[RobotInterface::getFeedbacks] Unable to get the feedback from the robot";
        return false;
    }

    if(m_useVelocityFilter)
    {
        // filter the joint position and the velocity
        m_velocityFeedbackDegFiltered = m_velocityFilter->filt(m_velocityFeedbackDeg);
        for(unsigned j = 0; j < m_actuatedDOFs; ++j)
            m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDegFiltered(j));
    }
    if(m_useWrenchFilter)
    {
        m_leftWrenchInputFiltered = m_leftWrenchFilter->filt(m_leftWrenchInput);
        m_rightWrenchInputFiltered = m_rightWrenchFilter->filt(m_rightWrenchInput);

        if(!iDynTree::toiDynTree(m_leftWrenchInputFiltered, m_leftWrench))
        {
            yError() << "[RobotInterface::getFeedbacks] Unable to convert left foot wrench.";
            return false;
        }
        if(!iDynTree::toiDynTree(m_rightWrenchInputFiltered, m_rightWrench))
        {
            yError() << "[RobotInterface::getFeedbacks] Unable to convert right foot wrench.";
            return false;
        }
    }
    return true;
}

bool RobotInterface::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if(!m_controlModeInterface)
    {
        yError() << "[RobotInterface::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if(!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError() << "[RobotInterface::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotInterface::setInteractionMode(yarp::dev::InteractionModeEnum interactionMode)
{
    std::vector<yarp::dev::InteractionModeEnum> interactionModes(m_actuatedDOFs, interactionMode);

    return setInteractionMode(interactionModes);
}

bool RobotInterface::setInteractionMode(std::vector<yarp::dev::InteractionModeEnum>& interactionModes)
{
    if(m_currentJointInteractionMode != interactionModes)
    {
        bool ok = m_interactionInterface->setInteractionModes(interactionModes.data());
        if (ok)
            m_currentJointInteractionMode = interactionModes;

        return ok;
    }

    return true;
}

bool RobotInterface::setPositionReferences(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                           const double& positioningTimeSec)
{
    if(m_controlMode != VOCAB_CM_POSITION)
    {
        if(!switchToControlMode(VOCAB_CM_POSITION))
        {
            yError() << "[RobotInterface::setPositionReferences] Unable to switch in position control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_POSITION;
    }

    m_positioningTime = positioningTimeSec;
    m_positionMoveSkipped = false;
    if(m_positionInterface == nullptr)
    {
        yError() << "[RobotInterface::setPositionReferences] Position I/F is not ready.";
        return false;
    }

    if(m_interactionInterface == nullptr)
    {
        yError() << "[RobotInterface::setPositionReferences] IInteractionMode interface is not ready.";
        return false;
    }

    if(m_impedanceControlInterface == nullptr)
    {
        yError() << "[RobotInterface::setPositionReferences] IImpedanceControlInterface interface is not ready.";
        return false;
    }

    m_desiredJointPositionRad = desiredJointPositionsRad;

    std::pair<int, double> worstError(0, 0.0);

    if(!getWorstError(desiredJointPositionsRad, worstError))
    {
        yError() << "[RobotInterface::setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second < 0.03)
    {
        m_positionMoveSkipped = true;
        return true;
    }

    if(positioningTimeSec < 0.01)
    {
        yError() << "[RobotInterface::setPositionReferences] The positioning time is too short.";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackDeg.data()))
    {
        yError() << "[RobotInterface::setPositionReferences] Error while reading encoders.";
        return false;
    }

    std::vector<double> refSpeeds(m_actuatedDOFs);

    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeUtilities::shortestAngularDistance(currentJointPositionRad,
                                                                                     desiredJointPositionsRad(i)));
        refSpeeds[i] = std::max(3.0, iDynTree::rad2deg(absoluteJointErrorRad) / positioningTimeSec);
    }

    if(!m_positionInterface->setRefSpeeds(refSpeeds.data()))
    {
        yError() << "[RobotInterface::setPositionReferences] Error while setting the desired speed of joints.";
        return false;
    }

    // convert a radians vector into a degree vector
    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(m_desiredJointPositionRad(i)) ;

    if(!m_positionInterface->positionMove(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotInterface::setPositionReferences] Error while setting the desired positions.";
        return false;
    }

    m_startingPositionControlTime = yarp::os::Time::now();
    return true;
}

bool RobotInterface::checkMotionDone(bool& motionDone)
{
    // if the position move is skipped the motion is implicitly done
    if(m_positionMoveSkipped)
    {
        motionDone = true;
        return true;
    }

    bool checkMotionDone = false;
    m_positionInterface->checkMotionDone(&checkMotionDone);

    std::pair<int, double> worstError;
    if (!getWorstError(m_desiredJointPositionRad, worstError))
    {
        yError() << "[RobotInterface::checkMotionDone] Unable to get the worst error.";
        return false;
    }

    double now = yarp::os::Time::now();
    double timeThreshold = 1;
    if (now - m_startingPositionControlTime > m_positioningTime + timeThreshold)
    {
        yError() << "[RobotInterface::checkMotionDone] The timer is expired but the joint "
                 << m_axesList[worstError.first] << " has an error of " << worstError.second
                 << " radians";
        return false;
    }

    motionDone = checkMotionDone && worstError.second < 0.1;
    return true;
}

bool RobotInterface::setDirectPositionReferences(const iDynTree::VectorDynSize& desiredPositionRad)
{
    if(m_positionDirectInterface == nullptr)
    {
        yError() << "[RobotInterface::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[RobotInterface::setDirectPositionReferences] Encoders I/F not ready.";
        return false;
    }

    if(m_controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        if(!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
        {
            yError() << "[RobotInterface::setDirectPositionReferences] Unable to switch in position-direct control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_POSITION_DIRECT;
    }

    if(desiredPositionRad.size() != m_actuatedDOFs)
    {
        yError() << "[RobotInterface::setDirectPositionReferences] Dimension mismatch between desired position "
                 << "vector and the number of controlled joints.";
        return false;
    }

    std::pair<int, double> worstError(0, 0.0);

    if(!getWorstError(desiredPositionRad, worstError))
    {
        yError() << "[RobotInterface::setDirectPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second > 0.5)
    {
        yError() << "[RobotInterface::setDirectPositionReferences] The worst error between the current and the "
                 << "desired position of the " <<  m_axesList[worstError.first]
                 << " joint is greater than 0.5 rad.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredPositionRad(i));

    if(!m_positionDirectInterface->setPositions(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotInterface::setDirectPositionReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotInterface::setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad)
{
    if(m_velocityInterface == nullptr)
    {
        yError() << "[RobotInterface::setVelocityReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[RobotInterface::setVelocityReferences] Encoders I/F not ready.";
        return false;
    }

    if(m_controlMode != VOCAB_CM_VELOCITY)
    {
        if(!switchToControlMode(VOCAB_CM_VELOCITY))
        {
            yError() << "[RobotInterface::setVelocityReferences] Unable to switch in velocity control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_VELOCITY;
    }

    if(desiredVelocityRad.size() != m_actuatedDOFs)
    {
        yError() << "[RobotInterface::setVelocityReferences] Dimension mismatch between desired velocity "
                 << "vector and the number of controlled joints.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredVelocityRad(i));

    if(!m_velocityInterface->velocityMove(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotInterface::setVelocityReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotInterface::close()
{
    m_rightWrenchPort.close();
    m_leftWrenchPort.close();
    switchToControlMode(VOCAB_CM_POSITION);
    m_controlMode = VOCAB_CM_POSITION;
    setInteractionMode(yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF);
    if(!m_robotDevice.close())
    {
        yError() << "[RobotInterface::close] Unable to close the device.";
        return false;
    }

    return true;
}

const iDynTree::VectorDynSize& RobotInterface::getJointPosition() const
{
    return m_positionFeedbackRad;
}
const iDynTree::VectorDynSize& RobotInterface::getJointVelocity() const
{
    return m_velocityFeedbackRad;
}

const iDynTree::Wrench& RobotInterface::getLeftWrench() const
{
    return m_leftWrench;
}

const iDynTree::Wrench& RobotInterface::getRightWrench() const
{
    return m_rightWrench;
}

const iDynTree::VectorDynSize& RobotInterface::getVelocityLimits() const
{
    return m_jointVelocitiesBounds;
}

const iDynTree::VectorDynSize& RobotInterface::getPositionUpperLimits() const
{
    return m_jointPositionsUpperBounds;
}

const iDynTree::VectorDynSize& RobotInterface::getPositionLowerLimits() const
{
    return m_jointPositionsLowerBounds;
}

const std::vector<std::string>& RobotInterface::getAxesList() const
{
    return m_axesList;
}

int RobotInterface::getActuatedDoFs()
{
    return m_actuatedDOFs;
}

WalkingPIDHandler& RobotInterface::getPIDHandler()
{
    return *m_PIDHandler;
}

const iDynTree::Transform& RobotInterface::getBaseTransform() const
{
    return m_robotBaseTransform;
}

const iDynTree::Twist& RobotInterface::getBaseTwist() const
{
    return m_robotBaseTwist;
}

void RobotInterface::setHeightOffset(const double& offset)
{
    m_heightOffset = offset;
}

bool RobotInterface::isExternalRobotBaseUsed()
{
    return m_useExternalRobotBase;
}

bool RobotInterface::loadCustomInteractionMode()
{
    return setInteractionMode(m_jointInteractionMode);
}

bool RobotInterface::setImpedanceControlGain()
{
    for (unsigned i = 0; i < m_actuatedDOFs; i++)
    {
        if(!m_impedanceControlInterface->setImpedance(i,m_stiffnessGainVector(i),m_dampingGainVector(i)))
        {
            yError() << "[RobotInterface::setImpedanceControlGain] Error while setting the impedance control gains";
            return false;
        }
    }

    return true;
}
