#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>

#include <serial/serial.h>
#include "sabertooth_2x25_driver.h"

#include "vicon/Subject.h"

#define pi 3.14159265


//Aspects of desired circular path
float radius = 1, Period = 20;
float thetadot = 2*pi/Period;


// Set radius of wheel and distance from CoM to wheel in meters
float rWheel = 0.04875;    // radius of wheel, m
float D = 0.175;         // distance from CoM to wheel, m


serial::Serial *ser;



/***********************************************************************************************/
/* SABERTOOTH FUNCTIONS */
/***********************************************************************************************/

// Write data to serial port
void writeToPort(char data){
    bool written=false;
    char* write = &data;
    while(!written){
        try{
            ser->write(write);
            written=true;
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




/***********************************************************************************************/
/* START MY FUNCTIONS */
/***********************************************************************************************/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    ros::NodeHandle nh;
    
    int freq = 100;
    ros::Rate loop_rate(freq);


    ser = new serial::Serial();
    ser->setPort("/dev/ttyS0");
    ser->open();
    ser->setBaudrate(9600);

    set_baudrate(2,128);
    set_baudrate(2,129);
    // std::cout<<"Start"<<std::endl;


    while(ros::ok()) {

        ser->flush();

        control_motors_sep(6,64,7,64,128);
        control_motors_sep(6,64,7,64,129);

        break;

        

        //Publish necessary topics
        ros::spinOnce();
        loop_rate.sleep();
        ser->flush();

        
    }
loop_rate.sleep();
control_motors_sep(6,64,7,64,128);
control_motors_sep(6,64,7,64,129);
ser->close();
return 0;
}