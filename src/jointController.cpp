#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>

#include "../include/kuka_control/planner.h"

using namespace std;
 
class KUKA_CONTROL {
	public:
		KUKA_CONTROL();
		void run();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();

	private:
		ros::NodeHandle _nh;
		ros::Subscriber _js_sub;
		ros::Publisher _js_pub;
		bool _first_js;
		KDL::JntArray *_q_in;
		KDL::JntArray *_dq_in;
		bool _sync;

};

KUKA_CONTROL::KUKA_CONTROL() {
	_js_sub = _nh.subscribe("/iiwa/joint_states", 0, &KUKA_CONTROL::joint_states_cb, this);
	_js_pub = _nh.advertise<std_msgs::Float64MultiArray>("/iiwa/jointsCommand", 0);

	_q_in = new KDL::JntArray( 7 );
	_dq_in = new KDL::JntArray( 7 );
	_first_js = false;
	_sync = false;
}


void KUKA_CONTROL::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) {
		_q_in->data[i] = js.position[i];
		_dq_in->data[i] = js.velocity[i];
	}

	_first_js = true;
	_sync = true;
}


void KUKA_CONTROL::ctrl_loop() {
	ros::Rate r(100);
	std_msgs::Float64MultiArray jcmd;
	jcmd.data.resize(7);

	cout<<"Control start"<<endl;
	while(!_first_js)
		usleep(1000);
	
	for(int i=0; i<7; i++) jcmd.data[i]=_q_in->data[i];
	
	float eps = 0.00001;
	cout<<"Got JS"<<endl;
	while( ros::ok() ) {
		cout<<"Publishing..."<<endl;
		jcmd.data[0] += eps;
		_js_pub.publish(jcmd);
		r.sleep();
	}

	cout<<endl<<"Finito"<<endl;
}


void KUKA_CONTROL::run() {
	boost::thread ctrl_loop_t ( &KUKA_CONTROL::ctrl_loop, this);
	ros::spin();
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "iiwa_control");

	KUKA_CONTROL kc;
	kc.run();

	return 0;
}
