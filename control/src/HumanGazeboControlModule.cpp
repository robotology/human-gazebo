#include "HumanGazeboControlModule.h"

const std::string LogPrefix = "HumanGazeboControlModule : ";

void HumanGazeboControlModule::addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list)
{
    prop.addGroup(key);
    yarp::os::Bottle & bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

bool HumanGazeboControlModule::configure(yarp::os::ResourceFinder& rf)
{
  if(!yarp::os::Network::initialized())
  {
    yarp::os::Network::init();
  }

  // ================
  // CHECK PARAMETERS
  // ================

  if(!rf.check("name"))
  {
    yError() << LogPrefix << "Module name is wrong or missing";
  }
  const std::string moduleName = rf.find("name").asString();
  rpc_port_name = "/" + moduleName + "/rpc:i";
  setName(moduleName.c_str());
  // Check that the name matches the module name in order to avoid
  // passing a wrong configuration file
  if (moduleName != "human-gazebo-control")
  {
    yError() << LogPrefix << "The moduleName parameter of the passed configuration is not human-gazebo-control";
    return false;
  }

  // MODULE PARAMETERS
  // =================

  if (!(rf.check("period") && rf.find("period").isInt()))
  {
    yError() << LogPrefix << "Parameter 'period' missing or invalid";
    return false;
  }

  // ROBOT PARAMETERS
  // =================

  if (!(rf.check("robot") && rf.find("robot").isString()))
  {
    yError() << LogPrefix << "Parameter 'robot' missing or invalid";
    return false;
  }

  // HDE PARAMETERS
  // =================

  if (!(rf.check("hde_state_port_name")))
  {
    yError() << LogPrefix << "Parameter 'hde_state_port_name' missing or invalid";
    return false;
  }

  // HUMAN PARAMETERS
  // =================

  if (!(rf.check("human_joints") && rf.find("human_joints").isInt()))
  {
    yError() << LogPrefix << "Parameter 'human_joints' missing or invalid";
    return false;
  }

  if (!(rf.check("human_joint_axes_list")))
  {
    yError() << LogPrefix << "Parameter 'human_joint_axes_list' missing or invalid";
    return false;
  }

  if (!(rf.check("human_joint_list")))
  {
    yError() << LogPrefix << "Parameter 'human_joint_list' missing or invalid";
    return false;
  }

  // ===============
  // READ PARAMETERS
  // ===============

  // MODULE PARAMETERS
  period = rf.find("period").asInt() / 1000.0;

  // ROBOT PARAMETERS
  robot = rf.find("robot").asString();

  // HDE PARAMETERS
  hde_state_port_name = rf.find("hde_state_port_name").asString();

  // HUMAN PARAMETERS
  human_joints = rf.find("human_joints").asInt();
  human_joint_axes_list = rf.findGroup("human_joint_axes_list");
  human_joint_list = rf.findGroup("human_joint_list");

  // =================================
  // INITIALIZE AND CONNECT YARP PORTS
  // =================================

  if(rpc_port.open(rpc_port_name))
  {
    attach(rpc_port);
  }

  if(state_port.open("/"+moduleName+"/state:i"))
  {
    if(!yarp::os::Network::connect(hde_state_port_name,state_port.getName().c_str()))
    {
      yError() << LogPrefix << "Failed to connect " << hde_state_port_name << " and " << state_port.getName().c_str();
      return false;
    }
  }
  else
  {
    yError() << LogPrefix << "Failed to open " << state_port.getName().c_str();
    return false;
  }

  //TODO read the joint names list and then put then in the control board options
  control_board_options.put("device","remotecontrolboardremapper");

  // ==========================
  // READ HUMAN JOINT NAME LIST
  // ==========================
  std::vector<std::string> axesList;
  if(!human_joint_axes_list.check("human_joint_axes_list"))
  {
      yError() << LogPrefix << "Failed to read human joint axes list";
      return false;
  }
  else
  {
      if(human_joint_axes_list.size()-1 != human_joints)
      {
          yError() << LogPrefix << "Mismatch between the joints number and joint axes list size from the config file";
          return false;
      }
      else
      {
          std::string axis_name;
          for(int i=0; i < human_joints; i++)
          {
              axis_name = human_joint_axes_list.get(i+1).asString();
              yInfo()<< "Axis name : " << axis_name;
              axesList.push_back(axis_name);
          }
      }
  }

  addVectorOfStringToProperty(control_board_options, "axesNames", axesList);
  yarp::os::Bottle remoteControlBoards;
  yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();

  if(!human_joint_list.check("human_joint_list"))
  {
      yError() << LogPrefix << "Failed to read human joint list";
      return false;
  }
  else
  {
      int nJoints = human_joints/3;
      if(human_joint_list.size()-1 != nJoints)
      {
          yError() << LogPrefix << "Mismatch between the joints number and joint name list size from the config file";
          return false;
      }
      else
      {;
          std::string joint_name;
          for(int i=0; i < nJoints; i++)
          {
              joint_name = human_joint_list.get(i+1).asString();
              yInfo() << LogPrefix << joint_name;
              remoteControlBoardsList.addString("/" + robot + "/" + joint_name);
          }
      }
  }

  yInfo() << LogPrefix << "Configuring control baords";

  control_board_options.put("remoteControlBoards",remoteControlBoards.get(0));
  control_board_options.put("localPortPrefix","/human-gazebo-control");

  yarp::os::Property & remoteControlBoardsOptions = control_board_options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
  remoteControlBoardsOptions.put("writeStrict","on");

  bool ok = controlDevice.open(control_board_options);
  if(!ok)
  {
      yError() << LogPrefix << "Failed to open remotecontrolboardremapper object";
      yError() << LogPrefix << "Verify the options passed to remotecontrolboardremapper";
      std::cerr << control_board_options.toString() << std::endl;
      return false;
  }

  // Obtain the interfaces
  ok = controlDevice.view(ienc);
  ok = ok && controlDevice.view(ipos);

  if(!ok)
  {
      yError() << LogPrefix << "Unable to open interfaces";
      return false;
  }

  yInfo() << LogPrefix << "Module configured successfully";
  return true;
}

void HumanGazeboControlModule::convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg)
{
    if (vecDeg.size() != vecRad.size()) {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i=0; i < vecRad.size(); i++) {
        vecDeg[i] = (180.0/M_PI)*vecRad[i];
    }
}

bool HumanGazeboControlModule::updateModule()
{
  //READ Human-state-provider
  human::HumanState *input_state = state_port.read();

  if(input_state->positions.size() == human_joints)
  {

      joint_positions_rad = input_state->positions;
      joint_positions_deg.resize(joint_positions_rad.size());

      int axes = 0;
      ipos->getAxes(&axes);
      if(axes != human_joints)
      {
          yError() << LogPrefix << "Error in the number of joints read";
          return false;
      }

      convertRadToDeg(joint_positions_rad,joint_positions_deg);

      bool motionDone = false;
      while(!motionDone)
      {
          yarp::os::Time::delay(0.1);
          ipos->checkMotionDone(&motionDone);
      }

      ipos->positionMove(joint_positions_deg.data());
  }
  else
  {
      yError() << LogPrefix << "DoFs mismatch between the config file and human state port";
      return false;
  }
  return true;
}

double HumanGazeboControlModule::getPeriod()
{
    return period;
}

bool HumanGazeboControlModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString()=="quit")
        return false;
    else
        reply=command;

    return true;
}

 bool HumanGazeboControlModule::interruptModule()
 {
     yInfo() << LogPrefix << "Interrupting module for port cleanup";

     controlDevice.close();

     // Interrupt yarp ports
     rpc_port.interrupt();
     state_port.interrupt();

     // Disconnect yarp ports
     yarp::os::Network::disconnect(hde_state_port_name,state_port.getName().c_str());

     return true;
}

bool HumanGazeboControlModule::close()
{
    yInfo() << LogPrefix << "Calling close function";

    controlDevice.close();

    // Close yarp ports
    rpc_port.close();
    state_port.close();

    return true;
}

HumanGazeboControlModule::~HumanGazeboControlModule()
{
    yarp::os::Network::fini();
}
