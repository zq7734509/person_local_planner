#ifndef PERSON_ZONE_COST_FUNCTION_H_
#define PERSON_ZONE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>


namespace base_local_planner {

class PersonZoneCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  PersonZoneCostFunction(nav_msgs::Odometry* robot_odom, nav_msgs::Odometry* person_odom,  Eigen::Matrix<double, 5, 1>* person_zone);

  virtual ~PersonZoneCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare();

  Eigen::Vector3f current_vel_;

private:

  nav_msgs::Odometry* p_robot_odom_;
  nav_msgs::Odometry* p_person_odom_;  
  Eigen::Matrix<double, 5, 1>* p_person_zone_;
  bool isInZone(Eigen::Matrix<double, 3, 1> endPoint, int& enable);
  double distanceToZone(int state, Eigen::Matrix<double, 3, 1> endPoint);
  int onWhichSide();
  bool nearZone_;
  double dis2zone_;
  int enable_;
  bool meet_;
  bool willMeet();
};

} /* namespace base_local_planner */
#endif /* PERSON_ZONE_COST_FUNCTION_H_ */
