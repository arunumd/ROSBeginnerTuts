#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "std_msgs/String.h"
#include <beginner_tutorials/service.h>

TEST(ServiceTest, service)
{
    ros::ServiceClient client = n->serviceClient<simple_rostest::service>(
                                    "change_string");
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    simple_rostest::service srv;
    srv.request.x = "History is getting updated";
    client.call(srv);

    EXPECT_STRNE(srv.response.y, srv.request.x);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testTalker");
    n.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
