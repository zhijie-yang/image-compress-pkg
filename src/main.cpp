//
// Created by yang on 2019/11/29.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <thread>

#include <sstream>


std::vector<ros::Publisher> publisher_vector;


void imageCallback (const sensor_msgs::ImageConstPtr& msg, const std::string& image_topic)
{
    /// TODO:
    /// Start a thread pool with #threads the system owns,
    /// Each time when this function is called, give the raw image to an idle thread
    /// Publish it out according to its topic when the thead returns
    /// Input topic "camera_n", where n is a number
    /// Output topic "camera_n_compressed", where n is a number
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_transporter");
    ros::NodeHandle nh;
    /// Gets the number of image topics
    int num_topics;
    nh.getParam(std::string("num_topics"), num_topics);
    ROS_ASSERT(num_topics >= 1);
    std::string cam_name = "camera_";
    /// Initializes a vector to hold all the subscribers
    std::vector<ros::Subscriber> subscriber_vector;
    subscriber_vector.resize(num_topics);
    /// Also resize the publisher vector and initializes its elements
    publisher_vector.resize(num_topics);
    for (int i = 1; i <= num_topics; ++i)
    {
        /// Converts and connects the number to the string of topic name
        std::string num;
        std::stringstream ss;
        ss << i;
        ss >> num;
        std::string image_topic = cam_name + num;
        std::string image_compressed = image_topic + "_compressed";
        /// TODO may change the second param into a number instead of a string
        subscriber_vector[i] = nh.subscribe<sensor_msgs::Image>(image_topic, 1, boost::bind(&imageCallback, _1, image_topic));
        publisher_vector[i] = nh.advertise<sensor_msgs::Image>(image_compressed, 1000);
    }

//    nh.getParam
//    ros::Subscriber sub = nh.subscribe("in_image_topic", 1, imageCallback);

    ros::spin();
}
