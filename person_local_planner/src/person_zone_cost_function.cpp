#include <person_local_planner/person_zone_cost_function.h>
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <Eigen/Dense>

namespace base_local_planner {

PersonZoneCostFunction::PersonZoneCostFunction(nav_msgs::Odometry* robot_odom, nav_msgs::Odometry* person_odom,  Eigen::Matrix<double, 5, 1>* person_zone) {

    // p_robotState_ = state;
    // p_recover_ = recover;
    // p_angle_ = angle;

    p_robot_odom_ = robot_odom;
    p_person_odom_ = person_odom;
    p_person_zone_ = person_zone;
    nearZone_ = false;
    enable_ = 1;
    dis2zone_ = 1000;
    current_vel_ << 0, 0, 0;
}

PersonZoneCostFunction::~PersonZoneCostFunction() {
}

bool PersonZoneCostFunction::prepare()
{
    bool meet = willMeet();
    ROS_INFO("enable:%d", enable_);
    //if(enable_ && meet)
    //{
        
    //    nearZone_ = false;
    //}
    if(enable_)
        nearZone_ = false;
    else
    {
        if(meet)
            nearZone_ = true;
        else
            nearZone_ = false;
    }
    enable_ = 1;        

    return true;
}

double PersonZoneCostFunction::scoreTrajectory(Trajectory &traj)
{
    double init_cost = 20000;     //将代价值置为一个比较大的值
    double final_cost = 0;
    double score;
    double dis_vel;
    double dis_vel_scale = 5000;
    double dis_zone_scale = 2000;

    nav_msgs::Odometry r_odom = *p_robot_odom_;
    nav_msgs::Odometry p_odom = *p_person_odom_;

    Eigen::Matrix<double, 3, 1> endPoint;  //轨迹末端在odom坐标系下的坐标（x,y,th）
    if(traj.getPointsSize())
    {
        traj.getEndpoint(endPoint(0), endPoint(1), endPoint(2));
        int side_state = onWhichSide();
        bool flag = isInZone(endPoint, enable_);     //判断轨迹末端是否在区域内
        //if(flag)
        //    nearZone_ = true;
        if(nearZone_ && !flag)
        {
            double dis_score;
            if(side_state == 1)  //在左侧，鼓励角速度为负
            {
                if(traj.thetav_ >= 0)
                    score = init_cost + 10;
                else
                {
                    dis_vel = 1 - fabs(traj.xv_ - 0.5);
                    dis2zone_ = distanceToZone(side_state, endPoint);
                    score = dis_vel * dis_vel_scale + (2 - dis2zone_) * dis_zone_scale;
                    if(dis2zone_ < 0)
                        score = init_cost + 10;
                    ROS_INFO("vx:%f, vth:%f, dis_score:%f, zone_score:%f", traj.xv_, traj.thetav_, dis_vel, dis2zone_);
                }
            }
            else if(side_state == 2)
            {
                if(traj.thetav_ <= 0)
                    score = init_cost + 10;
                else
                {
                    dis_vel = 1 - fabs(traj.xv_ - 0.5);
                    dis2zone_ = distanceToZone(side_state, endPoint);
                    score = dis_vel * dis_vel_scale + (2 - dis2zone_) * dis_zone_scale;
                    if(dis2zone_ < 0)
                        score = init_cost + 10;
                    ROS_INFO("vx:%f, vth:%f, dis_score:%f, zone_score:%f", traj.xv_, traj.thetav_, dis_vel, dis2zone_);
                }
            }
            else if(side_state == 3)
            {
                score = 0;
            }

            
        }
        else if(nearZone_ && flag)
        {
            score = init_cost + 10;
        }

        final_cost = init_cost - score;

    }
    return final_cost;    

}

//判断轨迹末端是否在区域内
bool PersonZoneCostFunction::isInZone(Eigen::Matrix<double, 3, 1> endPoint ,int& enable)
{
    bool flag = false;
    Eigen::Matrix<double, 5, 1> pz = *p_person_zone_;
    double x = (endPoint(0) - pz(2)) * cos(pz(4)) + (endPoint(1) - pz(3)) * sin(pz(4));
    double y = -(endPoint(0) - pz(2)) * sin(pz(4)) + (endPoint(1) - pz(3)) * cos(pz(4));
    double a = pz(0) /(sqrt(1-(pz(1))*(pz(1))));
    double b = pz(0);

    double result = (x / a) * (x / a) + (y / b) * (y / b) - 1;

    if(result<0){
      flag = true;
      enable = enable * 0;
    }
    else
    {
        flag = false;
        enable = enable * 1;   
    }
      
    
    return flag;
}

int PersonZoneCostFunction::onWhichSide()
{
    int state = 0;

    nav_msgs::Odometry r_odom = *p_robot_odom_;
    nav_msgs::Odometry p_odom = *p_person_odom_;

    tf::Quaternion p_quat;
    tf::quaternionMsgToTF(p_odom.pose.pose.orientation, p_quat);
    double roll, pitch, p_yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(p_quat).getRPY(roll, pitch, p_yaw);

    double vector_p1p2[3];
    double vector_p1r[3];
    double cross_z;
    double dot_result;

    vector_p1p2[0] = cos(p_yaw);
    vector_p1p2[1] = sin(p_yaw);

    vector_p1r[0] = r_odom.pose.pose.position.x - p_odom.pose.pose.position.x;
    vector_p1r[1] = r_odom.pose.pose.position.y - p_odom.pose.pose.position.y;

    cross_z = vector_p1p2[0] * vector_p1r[1] - vector_p1p2[1] * vector_p1r[0];
    dot_result = vector_p1r[0] * vector_p1p2[0] + vector_p1r[1] * vector_p1p2[1];

    if(cross_z > 0 && dot_result > 0)
        state = 1;
    else if(cross_z <= 0 && dot_result > 0)
        state = 2;
    else
        state = 3;
    
    return state;
}

bool PersonZoneCostFunction::willMeet()
{
    bool meet = false;
    nav_msgs::Odometry r_odom = *p_robot_odom_;
    nav_msgs::Odometry p_odom = *p_person_odom_;
    tf::Quaternion p_quat,r_quat;
    tf::quaternionMsgToTF(p_odom.pose.pose.orientation, p_quat);
    tf::quaternionMsgToTF(r_odom.pose.pose.orientation, r_quat);
    double roll, pitch, p_yaw, r_yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(p_quat).getRPY(roll, pitch, p_yaw);
    tf::Matrix3x3(r_quat).getRPY(roll, pitch, r_yaw);
    double delta_yaw = fabs(p_yaw - r_yaw);
    if(delta_yaw > M_PI)
        delta_yaw = 2*M_PI - delta_yaw;
    ROS_INFO("delta_yaw: %f",delta_yaw);
    if(delta_yaw > M_PI/6 && delta_yaw < 5*M_PI/6)
    {
        double r_x = r_odom.pose.pose.position.x;   double r_y = r_odom.pose.pose.position.y;
        double p_x = p_odom.pose.pose.position.x;   double p_y = p_odom.pose.pose.position.y;

        double r_a = sin(r_yaw);    double r_b = -cos(r_yaw);
        double p_a = sin(p_yaw);    double p_b = -cos(p_yaw);

        double r_c = -r_a * r_x - r_b * r_y;
        double p_c = -p_a * p_x - p_b * p_y;

        double x_meet = (r_b * p_c - p_b * r_c) / (r_a * p_b - p_a * r_b);
        double y_meet = (p_a * r_c - r_a * p_c) / (r_a * p_b - p_a * r_b);

        double theta_r_meet = atan2(y_meet-r_y,x_meet-r_x);
        double theta_p_meet = atan2(y_meet-p_y,x_meet-p_x);

        if(fabs(theta_r_meet - r_yaw)<0.01 && fabs(theta_p_meet - p_yaw)<0.01)
        {
            ROS_INFO("meet");
            return true;
        }
        else
        {
            ROS_INFO("can not meet");
            return false;
        }
    }
    else
    {
        ROS_INFO("meet street");
        return true;
    }
}

double PersonZoneCostFunction::distanceToZone(int state, Eigen::Matrix<double, 3, 1> endPoint)
{
    Eigen::Matrix<double, 5, 1> pz = *p_person_zone_;
    double a = pz(0) /(sqrt(1-(pz(1))*(pz(1))));
    double b = pz(0);
    double x = pz(2);
    double y = pz(3);
    double theta = pz(4);
    Eigen::Matrix<double, 3, 3> zone_in_odom;
    zone_in_odom << cos(theta), -sin(theta), x,
                    sin(theta), cos(theta), y,
                    0, 0, 1;
    Eigen::Matrix<double, 3, 3> odom2zone = zone_in_odom.inverse();
    bool positive_enable;
    double distance;
    
    nav_msgs::Odometry r_odom = *p_robot_odom_;
    nav_msgs::Odometry p_odom = *p_person_odom_;
    tf::Quaternion r_quat, p_quat;
    double roll, pitch, th_r, th_p;
    Eigen::Matrix<double, 3, 3> robot_in_odom, robot_in_zone, person_in_odom, person_in_zone, endpoint_in_odom, endpoint_in_zone;

    //计算机器人在区域坐标系内的坐标和朝向
    double x_r = r_odom.pose.pose.position.x;
    double y_r = r_odom.pose.pose.position.y;
    tf::quaternionMsgToTF(r_odom.pose.pose.orientation, r_quat);
    tf::Matrix3x3(r_quat).getRPY(roll, pitch, th_r);
    robot_in_odom << cos(th_r), -sin(th_r), x_r,
                     sin(th_r), cos(th_r), y_r,
                     0, 0, 1;
    robot_in_zone = odom2zone * robot_in_odom;
    double r_th_in_zone = acos(robot_in_zone(0,0));
    if(robot_in_zone(1,0)<0)
        r_th_in_zone = r_th_in_zone - M_PI;

    //计算行人在区域坐标系内的坐标和朝向
    tf::quaternionMsgToTF(p_odom.pose.pose.orientation, p_quat);
    tf::Matrix3x3(p_quat).getRPY(roll, pitch, th_p);
    person_in_odom << cos(th_p), -sin(th_p), 0,
                    sin(th_p), cos(th_p), 0,
                    0, 0, 1;
    person_in_zone = odom2zone * person_in_odom;
    double p_th_in_zone = acos(person_in_zone(0,0));
    if(person_in_zone(1,0)<0)
        p_th_in_zone = p_th_in_zone - M_PI;
    
    //计算末端点在区域坐标系内的坐标和朝向
    endpoint_in_odom << 1, 0, endPoint(0),
                        0, 1, endPoint(1),
                        0, 0, 1;
    endpoint_in_zone = odom2zone * endpoint_in_odom;
    double endpoint_th_in_zone = atan2(endpoint_in_zone(1,2), endpoint_in_zone(0,2));
   
    if((p_th_in_zone > -M_PI/12 && p_th_in_zone < M_PI/12) || (p_th_in_zone < -11*M_PI/12 && p_th_in_zone > 11*M_PI/12))
    {
        //对面相遇
        distance = sqrt((a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2))*(a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2)) + 
            (b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2))*(b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2)));
    }
    else
    
    {
        //通过行人和机器人的位置关系判断哪侧椭圆为有效区
        //state=1，则机器人在行人左侧；state=2，则机器人在行人右侧；state=3，机器人在行人背后

/*
        if(p_th_in_zone < 0)
        {
            if(state == 1)
                positive_enable =
            else if(state == 2)
                positive_enable =
        }
        else
        {
            if(state == 1)
                positive_enable =
            else if(state == 2)
                positive_enable =
        }
*/
        if(state == 1)
        {
            if(p_th_in_zone > 0)
            {
                positive_enable = false;
                ROS_INFO("DOWN");
            }
            else
            {
                positive_enable = true;
                ROS_INFO("UP");
            }
        }
                
        else if(state == 2)
        {
            if(p_th_in_zone > 0)
            {
                positive_enable = false;
                ROS_INFO("DOWN");
            }
            else
            {
                positive_enable = true;
                ROS_INFO("UP");
            }

        }
                


        if(positive_enable)
        {
            if(endpoint_th_in_zone < 0)
                distance = -1;
            else
            {
                distance = sqrt((a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2))*(a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2)) + 
                (b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2))*(b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2)));
            }
        }
        else
        {
            if(endpoint_th_in_zone >= 0)
                distance = -1;
            else
            {
                distance = sqrt((a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2))*(a*cos(endpoint_th_in_zone)-endpoint_in_zone(0,2)) + 
                (b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2))*(b*sin(endpoint_th_in_zone)-endpoint_in_zone(1,2)));
            }
        }
    }
    

    return distance;
    
}
/*
bool PersonZoneCostFunction::isRecover(int current_state)
{
    if((last_state_ == 1 || last_state_ == 2) && (current_state == 3 || current_state == 0)){
        ROS_INFO("laststate: %d, currentstate: %d",last_state_,current_state);
        last_state_ = current_state;
        
        return true;
    }
    else{
        last_state_ = current_state;
        return false;
    }
        
}
*/
} /* namespace base_local_planner */
