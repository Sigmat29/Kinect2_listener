#include "MyFunctions.h"
#include "geometry_msgs/Twist.h"


geometry_msgs::Twist changeTwist(float x, float y, float z, float turn)
{
    geometry_msgs::Twist msg_vel;
    msg_vel.angular.x = 0;
    msg_vel.angular.y = 0;
    msg_vel.angular.z = turn;
    msg_vel.linear.x = x;
    msg_vel.linear.y = y;
    msg_vel.linear.z = z;
    return(msg_vel);
}


void adjust (void)
{
    std_srvs::Empty srvflattrim;
    serviceflattrim.call(srvflattrim);
}
void emergency_function (void)
{
    std_msgs::Empty empty;
    emergency.publish(empty);
}
void takeoff (void)
{
    std_msgs::Empty empty;
    geometry_msgs::Twist msg_vel;
    topictakeoff.publish(empty);
    cout<<">>>Start<<<"<<endl;
    usleep(250000);
    msg_vel = changeTwist(0,0,0,0);
    cmd_vel.publish(msg_vel);
}
void land (void)
{
    std_msgs::Empty empty;
    topiclanding.publish(empty);
}

void forward (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(velocity,0,0,0);
    cmd_vel.publish(msg_vel);
}
void left (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,velocity,0,0);
    cmd_vel.publish(msg_vel);
}
void back (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-velocity,0,0,0);
    cmd_vel.publish(msg_vel);
}
void right (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,-velocity,0,0);
    cmd_vel.publish(msg_vel);
}
void up (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,velocity,0);
    cmd_vel.publish(msg_vel);
}
void down (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,-velocity,0);
    cmd_vel.publish(msg_vel);
}
void stop (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,0,0);
    cmd_vel.publish(msg_vel);
    //printf("stopping...\n");
}
//////////////////////////////////////////////////////////////////////////////////////////////
void up_left (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,velocity,velocity,0);
    cmd_vel.publish(msg_vel);
}
void up_right (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,-velocity,velocity,0);
    cmd_vel.publish(msg_vel);
}
void down_right (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,-velocity,-velocity,0);
    cmd_vel.publish(msg_vel);
}
void down_left (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,velocity,-velocity,0);
    cmd_vel.publish(msg_vel);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
void back_left (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-velocity,velocity,0,0);
    cmd_vel.publish(msg_vel);
}
void back_right (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-velocity,-velocity,0,0);
    cmd_vel.publish(msg_vel);
}
void forward_right (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(velocity,-velocity,0,0);
    cmd_vel.publish(msg_vel);
}
void forward_left (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(velocity,velocity,0,0);
    cmd_vel.publish(msg_vel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void up_back (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-velocity,0,velocity,0);
    cmd_vel.publish(msg_vel);
}
void up_forward (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(velocity,0,velocity,0);
    cmd_vel.publish(msg_vel);
}
void down_back (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-velocity,0,-velocity,0);
    cmd_vel.publish(msg_vel);
}
void down_forward (void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(velocity,0,-velocity,0);
    cmd_vel.publish(msg_vel);
}
void turn (void)
{
    geometry_msgs::Twist msg_vel; //info velocity
    msg_vel = changeTwist(0,0,0,1);
    cmd_vel.publish(msg_vel);
}






int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}
