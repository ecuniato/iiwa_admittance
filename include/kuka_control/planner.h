#include "ros/ros.h"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace Eigen;

bool T_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
bool Tdot_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
Matrix3d Skew(Vector3d v);
Vector3d Vee(Matrix3d S);
void twist2Vector(const geometry_msgs::TwistStamped twist, VectorXd& vel);
void accel2Vector(const geometry_msgs::AccelStamped acc, VectorXd& a);
void wrench2Vector(const geometry_msgs::WrenchStamped wrench, VectorXd& w);

class SPLINE_PLANNER {
	public:
		SPLINE_PLANNER(double freq);
    void compute_traj();
    void set_waypoints(std::vector<double> points, std::vector<double> times,double xdi=0,double xdf=0, double xddi=0, double xddf=0);
		bool isReady() {return _ready;};
		bool getNext(double &x, double &xd, double &xdd);

    std::vector<double> _x;
    std::vector<double> _xd;
    std::vector<double> _xdd;
    std::vector<double> _t;

	private:
    std::vector<double> _points;
    std::vector<double> _times;
    double _freq;
    int _N;
		bool _ready;
		int _counter;
		double _xdi,_xdf,_xddi,_xddf;

};

class CARTESIAN_PLANNER {
	public:
		CARTESIAN_PLANNER(double freq) : xplanner(freq),yplanner(freq),zplanner(freq),aplanner(freq) {_ready=false;_counter=0;_xdi.resize(6);_xdf.resize(6);_xddi.resize(6);_xddf.resize(6);};
    void compute();
    void set_waypoints(std::vector<geometry_msgs::PoseStamped> poses, std::vector<double> times);
		void set_waypoints(std::vector<geometry_msgs::PoseStamped> poses, std::vector<double> times, Eigen::VectorXd xdi,Eigen::VectorXd xdf, Eigen::VectorXd xddi, Eigen::VectorXd xddf);
		bool isReady() {return _ready;};
		bool getNext(geometry_msgs::PoseStamped &x, geometry_msgs::TwistStamped &xd, geometry_msgs::AccelStamped &xdd);

		std::vector<geometry_msgs::PoseStamped> _x;
    std::vector<geometry_msgs::TwistStamped> _xd;
    std::vector<geometry_msgs::AccelStamped> _xdd;
		std::vector<double> _t;

	private:
		void R_axisAngle(double th, Vector3d r, Matrix3d &R);
    std::vector<geometry_msgs::PoseStamped> _poses;
    std::vector<double> _times;
		SPLINE_PLANNER xplanner,yplanner,zplanner,aplanner;
		int _N;
		bool _ready;
		int _counter;
		Eigen::VectorXd _xdi,_xdf,_xddi,_xddf;
};
