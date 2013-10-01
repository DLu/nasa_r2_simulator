#include "gazebo_interface/GazeboInterface.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>

#include <r2_msgs/JointStatusArray.h>

using namespace gazebo;

GazeboInterface::GazeboInterface()
: ModelPlugin()
, advancedMode(false)
{
}

GazeboInterface::~GazeboInterface()
{
    // shutdown ros
    rosNodePtr->shutdown();
}

void GazeboInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    modelPtr = _model;

    prevStatesUpdateTime = _model->GetWorld()->GetSimTime();
    prevStatusUpdateTime = _model->GetWorld()->GetSimTime();

    // create controller
    robotControllerPtr.reset(new RobotController(modelPtr));

    // load parameters
    std::string robotNamespace = "";
    if (_sdf->HasElement("robotNamespace"))
    {
        robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
    }

    std::string paramsNamespace = "";
    if (_sdf->HasElement("paramsNamespace"))
    {
        paramsNamespace = _sdf->GetElement("paramsNamespace")->GetValueString() + "/";
    }

    if (!_sdf->HasElement("jointCommandsTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointCommandsTopic>, cannot proceed");
        return;
    }
    else
        jointCommandsTopic = _sdf->GetElement("jointCommandsTopic")->GetValueString();

    if (!_sdf->HasElement("jointStatesTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointStatesTopic>, cannot proceed");
        return;
    }
    else
        jointStatesTopic = _sdf->GetElement("jointStatesTopic")->GetValueString();

    if (!_sdf->HasElement("jointStatesRate"))
    {
        jointStatesStepTime = 0.;
    }
    else
        jointStatesStepTime = 1./_sdf->GetElement("jointStatesRate")->GetValueDouble();

    if (!_sdf->HasElement("advancedMode"))
    {
        ROS_INFO("GazeboInterface plugin missing <advancedMode>, defaults to false");
        advancedMode = false;
    }
    else
    {
        std::string strVal = _sdf->GetElement("advancedMode")->GetValueString();
        if (strVal == "1")
            advancedMode = true;
        else if (strVal == "0")
            advancedMode = false;
        else
        {
            ROS_WARN("GazeboInterface plugin <advancedMode> should be type bool (true/false), defaults to false");
            advancedMode = false;
        }
    }

    ROS_INFO("GazeboInterface plugin advancedMode: %s", advancedMode ? "true" : "false");

    if (advancedMode)
    {
        if (!_sdf->HasElement("jointControlTopic"))
        {
            ROS_FATAL("GazeboInterface plugin missing <jointControlTopic>, cannot proceed");
            return;
        }
        else
            jointControlTopic = _sdf->GetElement("jointControlTopic")->GetValueString();

        if (!_sdf->HasElement("jointStatusTopic"))
        {
            ROS_FATAL("GazeboInterface plugin missing <jointStatusTopic>, cannot proceed");
            return;
        }
        else
            jointStatusTopic = _sdf->GetElement("jointStatusTopic")->GetValueString();

        if (!_sdf->HasElement("jointStatusRate"))
        {
            jointStatusStepTime = 0.;
        }
        else
            jointStatusStepTime = 1./_sdf->GetElement("jointStatusRate")->GetValueDouble();
    }

    // create ros nodes
    rosNodePtr.reset(new ros::NodeHandle(robotNamespace));
    paramsNodePtr.reset(new ros::NodeHandle(paramsNamespace));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnectionPtr = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboInterface::update, this));

    ROS_INFO("Gazebo Interface plugin loaded");
}

void GazeboInterface::traverseParams(XmlRpc::XmlRpcValue param, std::map<std::string, XmlRpc::XmlRpcValue>& valMap, std::string searchKey,
                    std::string ns, std::string name)
{
    std::string fullName;
    if (name.empty())
    {
        fullName = ns;
    }
    else
    {
        fullName = ns + "/" + name;
    }

    if (param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        // if it is a struct, recurse
        for (XmlRpc::XmlRpcValue::iterator paramIt = param.begin(); paramIt != param.end(); ++paramIt)
        {
            traverseParams(paramIt->second, valMap, searchKey, fullName, paramIt->first);
        }
    }
    else if (searchKey == "")
    {
        // if there is no search key, add to map with fully qualified name
        valMap.insert(std::make_pair(fullName, param));
    }
    else if (searchKey == name)
    {
        // if the searchKey matches the name, add to map omitting searchKey
        valMap.insert(std::make_pair(ns, param));
    }

    // if searchKey is not matched, the function does nothing
}

bool GazeboInterface::getDoubleVal(XmlRpc::XmlRpcValue& val, double& doubleVal)
{
    if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        doubleVal = (double)(int)val;
    }
    else if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        doubleVal = (double)val;
    }
    else
    {
        return false;
    }

    return true;
}

void GazeboInterface::Init()
{
    ROS_DEBUG("add joints");
    // add all joints to controller
    for (unsigned int i = 0; i < modelPtr->GetJointCount(); ++i)
    {
        physics::JointPtr jPtr = modelPtr->GetJoints()[i];
        robotControllerPtr->addJoint(jPtr, advancedMode);
        ROS_DEBUG("Add %s to controller", jPtr->GetName().c_str());
    }

    XmlRpc::XmlRpcValue param;
    // set joint dependencies
    if (paramsNodePtr->getParam("dependency", param))
    {
        ROS_DEBUG("set dependencies");
        // get dependent children
        std::map<std::string, XmlRpc::XmlRpcValue> dependentChildrenMap;
        traverseParams(param, dependentChildrenMap, "children");
        // get dependent factors
        std::map<std::string, XmlRpc::XmlRpcValue> dependentFactorsMap;
        traverseParams(param, dependentFactorsMap, "factors");

        // iterate through and add dependencies
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator depIt = dependentChildrenMap.begin();
             depIt != dependentChildrenMap.end(); ++depIt)
        {
            XmlRpc::XmlRpcValue factorVal = dependentFactorsMap[depIt->first];
            // value can be a string or an array of strings
            if (depIt->second.getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                double val;
                if (getDoubleVal(factorVal, val))
                {
                    // child, parent, factor
                    ROS_DEBUG("Add joint dependency: %s, %s, %f", ((std::string)depIt->second).c_str(), depIt->first.c_str(), (double)factorVal);
                    robotControllerPtr->addJointDependency((std::string)depIt->second, depIt->first, (double)factorVal);
                }
                else
                {
                    ROS_WARN("Add dependency for %s (child) to %s (parent) not completed because factor not an appropriate type (double or int)",
                             ((std::string)depIt->second).c_str(), depIt->first.c_str());
                }
            }
            else if (depIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                if (factorVal.getType() == XmlRpc::XmlRpcValue::TypeArray && factorVal.size() == depIt->second.size())
                {
                    for (int i = 0; i < depIt->second.size(); ++i)
                    {
                        if (depIt->second[i].getType() == XmlRpc::XmlRpcValue::TypeString)
                        {
                            double val;
                            if (getDoubleVal(factorVal[i], val))
                            {
                                // child, parent, factor
                                ROS_DEBUG("Add joint dependency: %s, %s, %f", ((std::string)depIt->second[i]).c_str(), depIt->first.c_str(), val);
                                robotControllerPtr->addJointDependency((std::string)depIt->second[i], depIt->first, val);
                            }
                            else
                            {
                                ROS_WARN("Add dependency for %s (child) to %s (parent) not completed because factor not an appropriate type (double or int)",
                                         ((std::string)depIt->second[i]).c_str(), depIt->first.c_str());
                            }
                        }
                        else
                        {
                            ROS_WARN("A dependency to %s (parent) not added because type is invalid", depIt->first.c_str());
                        }
                    }
                }
                else
                {
                    ROS_WARN("Add dependency to %s (parent) not completed because children/factors array size not the same",
                             depIt->first.c_str());
                }
            }
        }
    }

    // get initial positions
    if (paramsNodePtr->getParam("initial_position", param))
    {
        ROS_DEBUG("set initial positions");
        bool radians = true;
        if (param.hasMember("radians") && param["radians"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            radians = (bool)param["radians"];
            ROS_DEBUG("radians: %d", radians);
        }
        else
        {
            ROS_WARN("No 'radians' member provided (or invalid type) in 'initial_position'; assumed true");
        }

        std::map<std::string, XmlRpc::XmlRpcValue> initialPoseMapParams;
        traverseParams(param, initialPoseMapParams);
        // remove '/radians'
        initialPoseMapParams.erase("/radians");

        // get positions
        std::map<std::string, double> initialPoseMap;
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator poseIt = initialPoseMapParams.begin();
             poseIt != initialPoseMapParams.end(); ++poseIt)
        {
            double val;
            if (getDoubleVal(poseIt->second, val))
            {
                if (radians == false)
                {
                    val = GZ_DTOR(val);
                }

                if (modelPtr->GetJoint(poseIt->first))
                {
                    initialPoseMap.insert(std::make_pair(poseIt->first, val));
                    ROS_DEBUG("initial position of %s set to %f", poseIt->first.c_str(), val);
                }
                else
                {
                    ROS_INFO("initial position of %s ignored because model doesn't contain joint", poseIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("Sim initial position value (%s) ignored because it is not a valid type (double or int)", poseIt->first.c_str());
            }
        }

        // set positions
        robotControllerPtr->setJointPositions(initialPoseMap);
    }

    // get position PIDsmodelPtr->GetWorld()->GetSimTime()
    if (paramsNodePtr->getParam("position_pid", param))
    {
        ROS_DEBUG("set position PIDs");
        std::map<std::string, XmlRpc::XmlRpcValue> positionPidMap;
        traverseParams(param, positionPidMap);

        // iterate through and add PIDs
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator pidIt = positionPidMap.begin();
             pidIt != positionPidMap.end(); ++pidIt)
        {
            if (pidIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray && pidIt->second.size() >= 3)
            {
                std::vector<double> gains(7, 0.0);
                bool valid = true;
                for (int i = 0; i < pidIt->second.size(); ++ i)
                {
                    valid &= getDoubleVal(pidIt->second[i], gains[i]);
                }

                if (valid)
                {
                    ROS_DEBUG("setPosPid: %s, %f, %f, %f, %f, %f, %f, %f", pidIt->first.c_str(), gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                    robotControllerPtr->setPosPid(pidIt->first, gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                }
                else
                {
                    ROS_WARN("position PID for %s not set because the values are not valid (double or int)", pidIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("position PID values for %s not set because it is not an array of size 3 or greater", pidIt->first.c_str());
            }
        }
    }

    // get velocity PIDs
    if (paramsNodePtr->getParam("velocity_pid", param))
    {
        ROS_DEBUG("set velocity PIDs");
        std::map<std::string, XmlRpc::XmlRpcValue> velocityPidMap;
        traverseParams(param, velocityPidMap);

        // iterate through and add PIDs
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator pidIt = velocityPidMap.begin();
             pidIt != velocityPidMap.end(); ++pidIt)
        {
            if (pidIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray && pidIt->second.size() >= 3)
            {
                std::vector<double> gains(7, 0.0);
                bool valid = true;
                for (int i = 0; i < pidIt->second.size(); ++ i)
                {
                    valid &= getDoubleVal(pidIt->second[i], gains[i]);
                }

                if (valid)
                {
                    ROS_DEBUG("setVelPid: %s, %f, %f, %f, %f, %f, %f, %f", pidIt->first.c_str(), gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                    robotControllerPtr->setVelPid(pidIt->first, gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                }
                else
                {
                    ROS_WARN("velocity PID for %s not set because the values are not valid (double or int)", pidIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("velocity PID values for %s not set because it is not an array of size 3 or greater", pidIt->first.c_str());
            }
        }
    }

    // if not in advanced mode, tell the controller to hold the joint positions
    if (!advancedMode)
    {
        for (unsigned int i = 0; i < modelPtr->GetJointCount(); ++i)
        {
            physics::JointPtr jPtr = modelPtr->GetJoints()[i];
            robotControllerPtr->setJointPosTarget(jPtr->GetName(), jPtr->GetAngle(0).Radian());
        }
    }

    // subscribe to ROS joint commands
    jointCommandsSub  = rosNodePtr->subscribe(jointCommandsTopic,  5, &GazeboInterface::commandJoints,  this);

    // advertise joint state publishing
    jointStatePub = rosNodePtr->advertise<sensor_msgs::JointState>(jointStatesTopic, true);

    if (advancedMode)
    {
        // subscribe to ROS joint control messages
        jointControlSub  = rosNodePtr->subscribe(jointControlTopic,  5, &GazeboInterface::controlJoints,  this);

        // advertise joint status publishing
        jointStatusPub = rosNodePtr->advertise<r2_msgs::JointStatusArray>(jointStatusTopic, true);
    }

    ROS_INFO("Gazebo Interface plugin initialized");
}

void GazeboInterface::update()
{
    common::Time currTime = modelPtr->GetWorld()->GetSimTime();

    // update joint controller
    robotControllerPtr->update();

    //publish joint states
    if ((currTime - prevStatesUpdateTime).Double() >= jointStatesStepTime)
    {
        prevStatesUpdateTime = currTime;
        sensor_msgs::JointStatePtr msgPtr(new sensor_msgs::JointState);
        msgPtr->header.stamp = ros::Time::now();
        // add all joints to jointState message
        for (unsigned int i = 0; i < modelPtr->GetJointCount(); ++i)
        {
            physics::JointPtr jPtr = modelPtr->GetJoints()[i];

            // joint
            std::string name = jPtr->GetName();
            msgPtr->name.push_back(name);
            msgPtr->position.push_back(jPtr->GetAngle(0).Radian());
            msgPtr->velocity.push_back(jPtr->GetVelocity(0));
            msgPtr->effort.push_back(jPtr->GetForce(0));

            // motor
            std::string::size_type index = name.find("/joint");
            if (index != std::string::npos)
            {
                msgPtr->name.push_back(name.replace(index, 6, "/motor"));
                msgPtr->position.push_back(jPtr->GetAngle(0).Radian());
                msgPtr->velocity.push_back(jPtr->GetVelocity(0));
                msgPtr->effort.push_back(jPtr->GetForce(0));

                // encoder
                msgPtr->name.push_back(name.replace(index, 6, "/encoder"));
                msgPtr->position.push_back(jPtr->GetAngle(0).Radian());
                msgPtr->velocity.push_back(jPtr->GetVelocity(0));
                msgPtr->effort.push_back(jPtr->GetForce(0));
            }
        }
        jointStatePub.publish(msgPtr);
    }

    //publish joint status
    if (advancedMode && (currTime - prevStatusUpdateTime).Double() >= jointStatusStepTime)
    {
        prevStatusUpdateTime = currTime;
        robotControllerPtr->publishJointStatuses(jointStatusPub);
    }
}

void GazeboInterface::commandJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
    bool setPos = msg->position.size() >= msg->name.size();
    bool setVel  = msg->velocity.size() >= msg->name.size();
    bool setEffort  = msg->effort.size() >= msg->name.size();

    ROS_DEBUG("GazeboInterface received joint command");
    for (unsigned int i = 0; i < msg->name.size(); ++i)
    {
        if (setPos)
        {
            robotControllerPtr->setJointPosTarget(msg->name[i], msg->position[i]);
        }
        if (setVel)
        {
            robotControllerPtr->setJointVelTarget(msg->name[i], msg->velocity[i]);
        }
        if (setEffort)
        {
            robotControllerPtr->setJointEffortTarget(msg->name[i], msg->effort[i]);
        }
    }
}

// handle control message
void GazeboInterface::controlJoints(const r2_msgs::JointControl::ConstPtr& msg)
{
    ROS_DEBUG("GazeboInterface received joint control");
    robotControllerPtr->setJointControl(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboInterface)
