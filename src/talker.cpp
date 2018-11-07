/************************************************************************************************
* @file      : Implementation file for ROS beginner tutorials turtlesim talker
* @author    : Arun Kumar Devarajulu
* @date      : October 28, 2018
* @copyright : 2018, Arun Kumar Devarajulu
* @license   : MIT License
*
*              Permission is hereby granted, free of charge, to any person obtaining a copy
*              of this software and associated documentation files (the "Software"), to deal
*              in the Software without restriction, including without limitation the rights
*              to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*              copies of the Software, and to permit persons to whom the Software is
*              furnished to do so, subject to the following conditions:
*
*              The above copyright notice and this permission notice shall be included in all
*              copies or substantial portions of the Software.
*
*              THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*              IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*              FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*              AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*              LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*              OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*              SOFTWARE.
*
* @brief     : The listener.cpp will be the listener node which publishes a custom message
**************************************************************************************************/
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/service.h"

// @brief  : We create a string object to publishg message
std::string message = "Message inserted by custom string ";

/****
*@brief  : Callback function for service call that changes the string contents
*@param  : req is the request type defined in the srv file
*@param  : res is the response type defined in the srv file
*@return : 'true' if it works as expected
***/
bool modifyContents(beginner_tutorials::service::Request &req,
                    beginner_tutorials::service::Response &res) {
    message = req.x; // 'x' is the input string for this service
    req.y = message; // 'y' is the output string for this service
    ROS_INFO_STREAM("Old message is being updated");
    return true;
}

int main(int argc, char **argv) {
    /******
     *@brief  : The ros::init() function needs to see argc and argv so that it can perform
     *          any ROS arguments and name remapping that were provided at the command line.
     *          For programmatic remappings you can use a different version of init() which takes
     *          remappings directly, but for most command-line programs, passing argc and argv is
     *          the easiest way to do it.  The third argument to init() is the name of the node.
     *
     *          You must call one of the versions of ros::init() before using any other
     *          part of the ROS system.
     ********/
    ros::init(argc, argv, "talker");

    /******
     *@brief  : NodeHandle is the main access point to communications with the ROS system.
     *          The first NodeHandle constructed will fully initialize this node, and the last
     *          NodeHandle destructed will close down the node.
     ********/
    ros::NodeHandle n;

    /******
     *@brief  : The advertise() function is how you tell ROS that you want to publish on a
     *          given topic name. This invokes a call to the ROS master node, which keeps a
     *          registry of who is publishing and who is subscribing. After this advertise()
     *          call is made, the master node will notify anyone who is trying to subscribe to
     *          this topic name, and they will in turn negotiate a peer-to-peer connection with
     *          this node. advertise() returns a Publisher object which allows you to publish
     *          messages on that topic through a call to publish(). Once all copies of the
     *          returned Publisher object are destroyed, the topic will be automatically unadvertised.
     *
     *          The second parameter to advertise() is the size of the message queue used for
     *          publishing messages.  If messages are published more quickly than we can send them,
     *          the number here specifies how many messages to buffer up before throwing some away.
     *******/
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // Creation of a ros service and advertising the node
    ros::ServiceServer service = nh.advertiseService("modifyContents", modifyContents);

    // The value for frequency is defined in 'beginner_tutorial.launch' file
    int freuency;
    ROS_INFO_STREAM("Set Publish frequency in Hz:" << 20);
    frequency = std::atoi(argv[1]);  // Assign the arg value to frequency

    ros::Rate loop_rate(10);

    /******
     *@brief  : A count of how many messages we have sent. This is used to create
     *          a unique string for each message.
     *******/
    int count = 0;
    while (ros::ok()) {
        /******
         *@brief  : This is a message object. You stuff it with data, and then publish it.
         *******/
        std_msgs::String msg;

        std::stringstream ss;
        ss << "This is my first Publisher message in ROS " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /******
         *@brief  : The publish() function is how you send messages. The parameter
         *          is the message object. The type of this object must agree with the type
         *          given as a template parameter to the advertise<>() call, as was done
         *          in the constructor above.
         *******/
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
