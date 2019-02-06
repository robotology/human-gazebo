#include <HumanGazeboControlModule.h>

int main(int argc, char **argv) {

    yarp::os::Network yarp_network;
    HumanGazeboControlModule module;
    yarp::os::ResourceFinder rf;

    rf.configure(argc,argv);

    if(!module.runModule(rf))
    {
        yError() << "HumanGazeboControlModule: Failed to run the module";
        return 1;
    }

    return 0;
}
