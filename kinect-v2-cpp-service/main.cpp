
#include "kinect2_impl.h"

int main(int argc, char* argv[])
{
	std::string dummy;
	

	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("sensors.kinect2");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 8888
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(8888);
	t->EnableNodeAnnounce(	RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the Kinect object
	boost::shared_ptr<Kinect2_impl> k = boost::make_shared<Kinect2_impl>();

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<sensors::kinect2::sensors__kinect2Factory>());

	// Register the Kinect object as a service
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("Kinect2", "sensors.kinect2", k);

	std::cout << "Connect to the Kinect v2 Service at: " << std::endl;
	std::cout << "tcp://localhost:8888/sensors.kinect2/Kinect2" << std::endl;
	std::cout << "Press enter to finish" << std::endl;
	std::getline(std::cin, dummy);
	k->ShutdownKinect();
	return 0;
}