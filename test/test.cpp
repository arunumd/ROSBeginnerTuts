#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "std_msgs/String.h"
#include <beginner_tutorials/service.h>

std::shared_ptr<ros::NodeHandle> n;

TEST(TESTTalker, service)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::service>(
                                    "service");
    beginner_tutorials::service srv;
    srv.request.x = "History is getting updated";
    client.call(srv);
    EXPECT_NE(srv.response.y, srv.request.x);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testTalker");
    n.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
