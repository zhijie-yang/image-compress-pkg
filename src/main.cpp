//
// Created by yang on 2019/11/29.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cstdint>
#include <turbojpeg.h>
#include <thread>
#include <boost/thread.hpp>
//#include <boost/thread/thread_pool.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <future>
#include "ThreadPool.h"

#define NUM_THREADS 32



std::vector<ros::Publisher> publisher_vector;
std::vector<unsigned long> publish_seq;
std::vector<std::vector<sensor_msgs::CompressedImagePtr>> compressed_buffer;


void insert_ordered (std::vector<sensor_msgs::CompressedImagePtr>& sub_buffer, const sensor_msgs::CompressedImagePtr& img)
{
    for (auto i = sub_buffer.begin(); i != sub_buffer.end(); ++i)
    {
        if (i->get()->header.seq > img->header.seq)
        {
            sub_buffer.insert(i, img);
            return;
        }
    }
    sub_buffer.push_back(img);
}

int trgb2jpeg(const unsigned char* rgb_buffer, int width, int height, int quality, unsigned char** jpeg_buffer, unsigned long* jpeg_size)
{
    tjhandle handle = nullptr;
    //unsigned long size=0;
    int flags = 0;
    int subsamp = TJSAMP_422;
    int pixelfmt = TJPF_RGB;

    handle=tjInitCompress();
    //size=tjBufSize(width, height, subsamp);
    tjCompress2(handle, rgb_buffer, width, 0, height, pixelfmt, jpeg_buffer, jpeg_size, subsamp,
                quality, flags);

    tjDestroy(handle);

    return 0;
}



void compressor_thread_func (const sensor_msgs::ImageConstPtr& msg, const int& camera_num)
{
    unsigned long jpegSize;
    unsigned char * jpegBuffer = nullptr;
    sensor_msgs::CompressedImage::Ptr img(new sensor_msgs::CompressedImage);
    trgb2jpeg(msg->data.data(), msg->width, msg->height, 1, &jpegBuffer, &jpegSize);
    img->data.assign(jpegBuffer, jpegBuffer+jpegSize);
    img->format = "jpeg";
    img->header.seq = msg->header.seq;
    img->header.frame_id = msg->header.frame_id;
    img->header.stamp = msg->header.stamp;

    if (publish_seq[camera_num] == img->header.seq - 1)
    {
        /// Publish directly
        publisher_vector[camera_num].publish(img);
        publish_seq[camera_num] = img->header.seq;
    } else
    {
        /// Into the buffer
        insert_ordered(compressed_buffer[camera_num], img);
    }

}


void imageCallback (const sensor_msgs::ImageConstPtr& msg, const int& camera_num, const ThreadPool& pool)
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
    int num_topics, num_threads;
    nh.getParam(std::string("num_topics"), num_topics);
    ROS_ASSERT(num_topics >= 1);
    num_threads = NUM_THREADS;
    nh.getParam(std::string("num_threads"), num_threads);
    ROS_ASSERT(num_topics >= 1);
    std::string cam_name = "camera_";
    /// Initializes a vector to hold all the subscribers
    std::vector<ros::Subscriber> subscriber_vector;
    subscriber_vector.resize(num_topics);
    /// Also resize the publisher vector and initializes its elements
    publisher_vector.resize(num_topics);
    publish_seq.resize(num_topics);
    compressed_buffer.resize(num_topics);

    ThreadPool pool(num_threads);


for (int i = 0; i < num_topics; ++i)
    {
        /// Converts and connects the number to the string of topic name
        std::string num;
        std::stringstream ss;
        ss << i;
        ss >> num;
        std::string image_topic = cam_name + num;
        std::string image_compressed = image_topic + "_compressed";
        publisher_vector[i] = nh.advertise<sensor_msgs::Image>(image_compressed, 1000);
        publish_seq[i] = 0;
        subscriber_vector[i] = nh.subscribe<sensor_msgs::Image>(image_topic, 1, boost::bind(&imageCallback, _1, i, pool));
    }

//    nh.getParam
//    ros::Subscriber sub = nh.subscribe("in_image_topic", 1, imageCallback);

    ros::spin();
}
