#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>

#include <SerialPort.h>
#include "sabertooth_2x25_driver.h"


#define pi 3.14159265


//Aspects of desired circular path
float radius = 2, T = 30;
float thetadot = 2*pi/T;

// Suscriber/Publisher
ros::Subscriber vicon_sub;
ros::Publisher vel_pub,ref_pub;


double x,y, theta;
double dXc,dYc,dthetaC,wheel1,wheel2,wheel3, wheel1speed, wheel2speed, wheel3speed;

// Set radius of wheel and distance from CoM to wheel in meters
float rWheel = 0.02;    // radius of wheel, m
float D = 0.0307807;         // distance from CoM to wheel, m

geometry_msgs::Pose2D reftraj;
ros::Time last_received;
std_msgs::Int16MultiArray twist;

SerialPort ser("/dev/ttyS0");

ros::Time beginTime;
ros::Duration runPeriod = ros::Duration(T);
ros::Time loopTime = beginTime + runPeriod;


double Xt(double time)
{
    return radius*cos(time*thetadot);
}

double Yt(double time)
{
    return radius*sin(time*thetadot);
}

double dXt(double time)
{
    return -radius*thetadot*sin(time*thetadot);
}

double dYt(double time)
{
    return radius*thetadot*cos(time*thetadot);
}



/*
void viconCallback(geometry_msgs::PoseStamped rover)
{
    
    x = rover.x;
    y = rover.y;
    theta = rover.theta + pi/6;
    last_received = ros::Time::now();
    
    
    //if (theta>=pi) theta -= 2*pi + pi/6;
    if (theta<=0) theta += 2*pi;
    
    //if ((last_received >= beginTime) & (theta<=0)) {
    //    theta += 2*pi;
    //}
    
    if (last_received > loopTime) theta += 2*pi;
    

    
    //x = rover.pose.position.x - 1;
    //y = rover.pose.position.y;
    //last_received = ros::Time::now();
    //tf::Quaternion q(rover.pose.orientation.x,rover.pose.orientation.y,rover.pose.orientation.z,rover.pose.orientation.w);
    //tf::Matrix3x3 m(q);
    //double roll,pitch,yaw;
    //m.getRPY(roll,pitch,yaw);
    //yaw = yaw - pi/2;
    //if (yaw>pi) yaw -= 2*pi;
    //if (yaw<-pi) yaw += 2*pi;
    
}
*/


int speedCalc(float wheel_velocity)
{

    float omega;
    float RPM;
    float speed;

    // Calculate angular rate of wheel in rad/s
    omega = wheel_velocity/rWheel;
    // Calculate RPM from angular rate
    RPM = 60*omega/(2*pi);

    if ((RPM >= -104) & (RPM < 0))
    {
        speed = 63*(RPM+104)/104 + 1;
    }
    if ((RPM > 0) & (RPM <= 104))
    {
        speed = 63*RPM/104 + 64;
    }
    if (RPM = 0)
    {
        speed = 64;
    }

    return speed = round(speed);
}




void writeToPort(char data){
    bool written=false;
    char* write = &data;
    //ser.flush();
    while(!written){
        try{
            ser.Write(write);
            written=true;
            //control_motors_sep(0,0,4,0,128);
        }
        catch(std::exception& e){
            continue;
        }
    }
}


/**********************************************************************************************
 * Function:        static void send_command ( uint8_t command, uint8_t value, uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives the command data from the driver functions
 * Output:          Sends the three commands plus their checksum to the serial port, and through
 *                      that to the Sabertooth Motor Controller
 * Side Effects:    None
 * Overview:        None
 * Notes:           Static helper function, for use only by driver functions in this file
 *********************************************************************************************/

    /* Helper Command, for internal driver use only
     * Defining it here, in the .c file, instead of in the .h file, to prevent
     * compiler warning about it being declared "static", but never defined
     * This is correct, since it is not for the user, only the user's functions */

    

/*********************************************************************************************/
static void send_command ( uint8_t command, uint8_t value, uint8_t address ) {
    //assert  ( command < COMMAND_HIGH_LIMIT);
    // putchar ( address );
    // putchar ( command );
    // putchar ( value );
    // putchar ( ( address + command + value ) & CRC_MASK );

    writeToPort( (char) address );
    writeToPort( (char) command );
    writeToPort( (char) value );
    writeToPort( (char) (( address + command + value ) & CRC_MASK) );
}



/**********************************************************************************************
 * Function:        uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
 *                                               uint8_t command2, uint8_t speed2, \
 *                                               uint8_t address )
 *
 * Pre-Condition:   None
 * Input:           Receives command data from the application program
 * Output:          Sends the commands to the send_command function,
 *                      and then to the serial port
 * Side Effects:    None
 * Overview:        Checks commands for validity, and passes them to the serial port
 * Notes:           This function is valid for Sabertooth Commands 0, 1, 4 - 7
 *                  These commands are for controlling two motors with individual settings, a single motor at a time
 *
 *                      Individual Motor Commands:
 *                          0:  Drive Forward      Motor 1
 *                          1:  Drive Reverse      Motor 1
 *                          4:  Drive Forward      Motor 2
 *                          5:  Drive Reverse      Motor 2
 *                          6:  Drive 7-Bit        Motor 1
 *                          7:  Drive 7-Bit        Motor 2
 *
 *********************************************************************************************/

uint8_t control_motors_sep ( uint8_t command1, uint8_t speed1, \
                             uint8_t command2, uint8_t speed2, \
                             uint8_t address ) {
// = MOTOR_DRIVER_ADDRESS_1 ) { // If your compiler allows overloading, feel free to un-comment the assignment
// and move it immediately after the "address" variable
    if ( ( command1 < COMMAND_LOW_LIMIT || command1 > DRIVE_MOTOR_2_7_BIT ) || \
         ( command2 < COMMAND_LOW_LIMIT || command2 > DRIVE_MOTOR_2_7_BIT ) ) {

   /*     Set error code for invalid command
    *     Call a user error function to do whatever your application requires, such as:
    *         mSTOP_ALL_MOTORS;  */
        return FALSE;
    }

    else {
      
        send_command ( command1, speed1, address );
        send_command ( command2, speed2, address );
//      Set error code for no error
        return TRUE;
    }
}

uint8_t set_baudrate ( uint8_t desired_baudrate, uint8_t address ) {

    static uint8_t new_baudrate = DEFAULT_BAUDRATE;

    if ( desired_baudrate < BAUDRATE_2400 || \
         desired_baudrate > BAUDRATE_38400 ) {

            new_baudrate = DEFAULT_BAUDRATE;
            return FALSE;                                   // Set error code for error
    }

    else {
        new_baudrate = desired_baudrate;
    }

    send_command ( SET_BAUD_RATE, new_baudrate, address );
    return TRUE;                                            // Set error code for no error
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    ros::NodeHandle nh;
    
    int freq = 10;
    ros::Rate loop_rate(freq);

    last_received = ros::Time::now();
    ros::Duration d;
    
    
    vel_pub = nh.advertise<std_msgs::Int16MultiArray>("/cmd",10);
    //ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ref_traj",10);
    ref_pub = nh.advertise<geometry_msgs::Pose2D>("/ref_traj",10);

    //vicon_sub = nh.subscribe("/open_base/pose/world", 100, viconCallback);


    double time, ref_time;
    double ref_theta;
 
    double J11,J12,J13,J21,J22,J23,J31,J32,J33;
    //reftraj.header.frame_id = "slam";
    double Pxerr=0.0,Pyerr=0.0,Ptheta_err=0.0,Iyerr=0.0,Ixerr=0.0,Itheta_err=0.0,Dxerr=0.0,Dyerr=0.0,Dtheta_err=0.0,OldErrX=0.0,OldErrY=0.0,OldErrTheta=0.0;

    //Parameters defining wheel location on body
    double alpha1, alpha2, alpha3, nu;
    //alpha is angle relative to local frame, nu is offset from x,y axis
    alpha1 = 0;
    alpha2 = 2*pi / 3;
    alpha3 = 4*pi / 3;
    nu = 0;



    ser.Open();
    ser.SetBaudRate(SerialPort::BAUD_9600);

    set_baudrate(2,128);
    set_baudrate(2,129);
    std::cout<<"Start"<<std::endl;
    
while(ros::ok()) {
    d = ros::Time::now() - last_received;
    time = d.toSec();

    ref_time = last_received.toSec();

    //Publish a reference trajectory to monitor on rviz
    //reftraj.pose.position.x = Xt(ref_time);
    //reftraj.pose.position.y = Yt(ref_time);
    //reftraj.pose.position.z = 0;
    //reftraj.pose.orientation.......

    reftraj.x = Xt(ref_time);
    reftraj.y = Yt(ref_time);

    ref_theta = ref_time*thetadot;
    //if (ref_theta > pi) ref_theta -= 2*pi;
    //if (ref_theta > 2*pi) ref_theta -= 2*pi;
    reftraj.theta = ref_theta;


    //PID controller
    // Proportional control:
    Pxerr = (Xt(ref_time) - x);
    Pyerr = (Yt(ref_time) - y);
    Ptheta_err = (ref_theta - theta);

    // Integral control:
    Ixerr += (Xt(ref_time) - x)/freq;
    Iyerr += (Yt(ref_time) - y)/freq;
    Itheta_err += (ref_theta - theta)/freq;
    if (Ixerr>0.5) {
        Ixerr = 0.5;
    }
    else if (Ixerr<-0.5) { 
        Ixerr = -0.5; 
    }
    if (Iyerr>0.5) {
        Iyerr = 0.5;
    }
    else if (Iyerr<-0.5) {
        Iyerr = -0.5;
    }
    if (Itheta_err>0.5) {
        Itheta_err = 0.5;
    }
    else if (Itheta_err<-0.5) {
        Itheta_err = -0.5;
    }

    // Derivative control:
    Dxerr = (Xt(ref_time) - x) - OldErrX;
    Dxerr = (Yt(ref_time) - y) - OldErrY;
    Dtheta_err = (ref_theta - theta) - OldErrTheta;

    OldErrX = Xt(ref_time) - x;
    OldErrY = Yt(ref_time) - y;
    OldErrTheta = ref_theta - theta;

    // Set P, I, D values
    double Kp, Ki, Kd;
    Kp = 0;
    Ki = 0;
    Kd = 0;

    //Calculate robot speed with PID feedback
    dXc = dXt(ref_time) + Kp*Pxerr + Ki*Ixerr + Kd*Dxerr;
    dYc = dYt(ref_time) + Kp*Pyerr + Ki*Iyerr + Kd*Dyerr;
    dthetaC = thetadot + Kp*Ptheta_err + Ki*Itheta_err + Kd*Dtheta_err;


    //Calculate wheel speeds with vanHaendel model
    // dXc is the xdot, dYc is the ydot, dthetaC is the thetadot

    //Make the Jinv matrix
    J11 = -sin(nu);
    J12 = cos(nu);
    J13 = D;
    J21 = -sin(nu+alpha2);
    J22 = cos(nu+alpha2);
    J23 = D;
    J31 = -sin(nu+alpha3);
    J32 = cos(nu+alpha3);
    J33 = D;

    wheel1 = J11*dXc + J12*dYc + J13*dthetaC;       
    wheel2 = J21*dXc + J22*dYc + J23*dthetaC;       
    wheel3 = J31*dXc + J32*dYc + J33*dthetaC;      


    wheel1speed = speedCalc(wheel1);
    wheel2speed = speedCalc(wheel2);
    wheel3speed = speedCalc(wheel3);


    if (wheel1speed < 1) wheel1speed = 1;
    if (wheel1speed > 127) wheel1speed = 127;

    if (wheel2speed < 1) wheel2speed = 1;
    if (wheel2speed > 127) wheel2speed = 127;

    if (wheel3speed < 1) wheel3speed = 1;
    if (wheel3speed > 127) wheel3speed = 127;


    control_motors_sep(6,wheel1speed,7,wheel2speed,128);
    control_motors_sep(6,64,7,wheel3speed,129);


    //Saturate wheel speeds and push into topic type
    /*
    if (wheel1 < 1) wheel1 = 1;
    if (wheel1 > 127) wheel1 = 127;

    if (wheel2 < 128) wheel2 = 128;
    if (wheel2 > 255) wheel2 = 255;

    if (wheel3 < 1) wheel3 = 1;
    if (wheel3 > 127) wheel3 = 127;
    */

    /*
    twist.data.clear();
    twist.data.push_back(wheel1);
    twist.data.push_back(wheel2);
    twist.data.push_back(wheel3);
    twist.data.push_back(0);
    */




    //Publish necessary topics
    ref_pub.publish(reftraj);
    ros::spinOnce();
    loop_rate.sleep();

    
 }



/*
while (ros::ok())
{



    omni_command.movement = 3;
    omni_command.wheel.v_left = -0.09827304;
    omni_command.wheel.v_back = 0.21588622;
    omni_command.wheel.v_right = -0.09827304;

    command_pub.publish(omni_command);

    ros::spinOnce();
    loop_rate.sleep();
}
*/
}