/************************************************************************************************
* @file      : Rostest file for Week 11 exercise
* @author    : Arun Kumar Devarajulu
* @date      : November 13, 2018
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
* @brief     : The test.cpp file tests the service 'modifyContents' in the /src/talker.cpp file
**************************************************************************************************/
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "std_msgs/String.h"
#include <beginner_tutorials/service.h>

/* Create a nodehandle shared pointer*/
std::shared_ptr<ros::NodeHandle> n;

TEST(TESTTalker, service)
{
    ros::NodeHandle n;
    /* Create a service client and test using not equal test case*/
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::service>(
                                    "service");
    beginner_tutorials::service srv;
    srv.request.x = "History is getting updated";
    client.call(srv);
    EXPECT_NE(srv.response.y, srv.request.x);

}

int main(int argc, char **argv)
{
	/* Code to run all tests*/
    ros::init(argc, argv, "testTalker");
    n.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
