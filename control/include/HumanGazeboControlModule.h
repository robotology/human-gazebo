#ifndef HUMANGAZEBOCONTROLMODULE_H
#define HUMANGAZEBOCONTROLMODULE_H

#include <iostream>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <../../human-dynamics-estimation/human-state-provider/thrifts/HumanState.h>
#include <../../human-dynamics-estimation/human-state-provider/thrifts/HumanStateProviderService.h>

class HumanGazeboControlModule:public yarp::os::RFModule
{
private:

  double period;
  std::string robot;
  std::string rpc_port_name;
  std::string hde_state_port_name;

  yarp::os::RpcServer rpc_port;
  yarp::os::BufferedPort<human::HumanState> state_port;

  int human_joints;
  yarp::os::Bottle human_joint_list;
  yarp::os::Bottle human_joint_axes_list;

public:

  yarp::os::Property control_board_options;
  yarp::dev::PolyDriver controlDevice;
  yarp::dev::IEncoders *ienc{nullptr};
  yarp::dev::IPositionControl *ipos{nullptr};

  yarp::sig::Vector joint_positions_rad;
  yarp::sig::Vector joint_positions_deg;

  HumanGazeboControlModule(){}
  ~HumanGazeboControlModule() override;

  void addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list);
  void convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg);

  bool configure(yarp::os::ResourceFinder& rf) override;
  bool updateModule() override;
  double getPeriod() override;
  bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) override;
  bool interruptModule() override;
  bool close() override;

};

#endif // HUMANGAZEBOCONTROLMODULE_H
