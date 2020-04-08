//
// Created by yang on 2019/11/29.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define NUM_THREADS 32
#define NUM_TOPICS 1

std::mutex wait_mtx;
std::vector<std::mutex> vec_comp_mtx;
//std::mutex* vec_comp_mtx;
std::condition_variable cond_var;


std::vector<ros::Publisher> publisher_vector;
std::vector<unsigned long> publish_seq;
std::vector<std::list<sensor_msgs::ImageConstPtr>> image_buffer;
std::vector<std::list<sensor_msgs::CompressedImage>> compressed_buffer;
std::mutex buf_mtx;


enum compressionFormat
{
    UNDEFINED = -1, JPEG, PNG
};

void check_and_publish()
    {
    for (int i = 0; i < compressed_buffer.size(); ++i)
    {
        std::unique_lock<std::mutex> guard(vec_comp_mtx[i]);
        if (compressed_buffer[i].size() ==
            compressed_buffer[i].end()->header.seq - publish_seq[i])
        {
            while (!compressed_buffer[i].empty())
            {
                publisher_vector[i].publish(compressed_buffer[i].front());
                compressed_buffer[i].pop_front();
            }
        }
    }
    }


/// This function is not thread-safe, lock before calling it
template<class T>
bool buffer_not_empty(std::vector<std::list<T>> v)
    {
    bool res = false;
    for (int i = 0; i < v.size(); ++i)
    {
        res |= !v[i].empty();
        if (res)
            return res;
    }
    return res;
    }

/// This function is not thread-safe, lock the buf_mtx before calling it
int longest_queue()
    {
    int longest = 0;
    for (int i = 0; i < image_buffer.size() - 1; ++i)
    {
        if (image_buffer[i + 1].size() > image_buffer[i].size())
            longest = i + 1;
    }
    return longest;
    }

/// This function is not thread-safe, lock the comp_mtx before calling it
void insert_ordered(std::list<sensor_msgs::CompressedImage> &sub_buffer, const sensor_msgs::CompressedImage &img)
    {
    for (auto i = sub_buffer.begin(); i != sub_buffer.end(); ++i)
    {
        if (i->header.seq > img.header.seq)
        {
            sub_buffer.insert(i, img);
            return;
        }
    }
    sub_buffer.push_back(img);
    }

void compressor_thread_func(const int& tid)
    {
    namespace enc = sensor_msgs::image_encodings;
    using namespace cv;


    while (1)
    {

        std::unique_lock<std::mutex> ulk(buf_mtx);
        int camera_num = longest_queue();
        if (!image_buffer[camera_num].empty())
        {
            sensor_msgs::ImageConstPtr msg = image_buffer[camera_num].front();
            image_buffer[camera_num].pop_front();
            ulk.unlock();

            sensor_msgs::CompressedImage compressed;
            compressed.header = msg->header;
            compressed.format = msg->encoding;

            // Compression settings
            std::vector<int> params;

            // Get codec configuration
            compressionFormat encodingFormat = JPEG;

            // Bit depth of image encoding
            int bitDepth = enc::bitDepth(msg->encoding);
//            int numChannels = enc::numChannels(msg->encoding);

            params.resize(9, 0);
            params[0] = IMWRITE_JPEG_QUALITY;
            params[1] = 90;
            params[2] = IMWRITE_JPEG_PROGRESSIVE;
            params[3] = 0;
            params[4] = IMWRITE_JPEG_OPTIMIZE;
            params[5] = 0;
            params[6] = IMWRITE_JPEG_RST_INTERVAL;
            params[7] = 0;

            // Update ros msg format header
            compressed.format += "; jpeg compressed ";

            // Check input format
            if ((bitDepth == 8) || (bitDepth == 16))
            {
                // Target image format
                std::string targetFormat;
                // printf("Check is color\n");
                if (enc::isColor(msg->encoding))
                {
                    // convert color images to BGR8 format
                    targetFormat = "bgr8";
                    compressed.format += targetFormat;
                }
                // printf("Try into bridge.\n");
                // OpenCV-ros bridge
                try
                {
//                            boost::shared_ptr<CompressedPublisher> tracked_object;
                    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, targetFormat);
                    // printf("Bridge succeeded.\n");
                    // Compress image
                    if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
                    {

                        float cRatio = (float) (cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                                       / (float) compressed.data.size();
                        ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)",
                                  cRatio, compressed.data.size());
                    }
                    else
                    {
                        ROS_ERROR("cv::imencode (jpeg) failed on input image");
                    }
                }
                catch (cv_bridge::Exception &e)
                {
                    ROS_ERROR("CV_BRIDGE EXCEPTION: %s", e.what());
                }
                catch (cv::Exception &e)
                {
                    ROS_ERROR("CV EXCEPTION %s", e.what());
                }

                // Publish message
//                        publish_fn(compressed);
            }
            else
                ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", msg->encoding.c_str());

            std::unique_lock<std::mutex> guard(vec_comp_mtx[camera_num]);

            publisher_vector[camera_num].publish(compressed);
        }
        else
        {
            ulk.unlock();
            std::unique_lock<std::mutex> lk(wait_mtx);
            cond_var.wait(lk, []() {
            std::unique_lock<std::mutex> guard(buf_mtx);
            return buffer_not_empty<sensor_msgs::ImageConstPtr>(image_buffer);
        });
        }
    }
    }


void imageCallback(const sensor_msgs::ImageConstPtr msg, const int &camera_num)
    {
    /// TODO:
    /// Start a thread pool with #threads the system owns,
    /// Each time when this function is called, give the raw image to an idle thread
    /// Publish it out according to its topic when the thead returns
    /// Input topic "camera_n", where n is a number
    /// Output topic "camera_n_compressed", where n is a number

    std::unique_lock<std::mutex> guard(buf_mtx);
    image_buffer[camera_num].push_back(msg);
    guard.unlock();
    cond_var.notify_all();
//    check_and_publish();
    }


int main(int argc, char **argv)
    {
    ros::init(argc, argv, "image_transporter");
    ros::NodeHandle nh;
    /// Gets the number of image topics
    int num_topics, num_threads;
    num_topics = NUM_TOPICS;
    nh.getParam(std::string("num_topics"), num_topics);
    ROS_ERROR("Initializing with %d cameras\n", num_topics);
    ROS_ASSERT(num_topics >= 1);
    num_threads = NUM_THREADS;
    nh.getParam(std::string("num_threads"), num_threads);
    ROS_ERROR("Initializing with %d threads\n", num_threads);
    ROS_ASSERT(num_threads >= 1);
    std::vector<std::thread> vec_thread;
    vec_thread.resize(num_threads);
    std::string cam_name = "camera_";
    /// Initializes a vector to hold all the subscribers
    std::vector<ros::Subscriber> subscriber_vector;
    subscriber_vector.resize(num_topics);
    /// Also resize the publisher vector and initializes its elements
    publisher_vector.resize(num_topics);
    publish_seq.resize(num_topics);
    compressed_buffer.resize(num_topics);
    image_buffer.resize(num_topics);
    std::vector<std::mutex> _vec_mtx(num_threads);
    std::mutex array_comp_mtx[num_threads];
    vec_comp_mtx.swap(_vec_mtx);

    for (int i = 0; i < num_topics; ++i)
    {
        /// Converts and connects the number to the string of topic name
        std::string num;
        std::stringstream ss;
        ss << i;
        ss >> num;
        std::string image_topic = cam_name + num + "/image_raw";
        std::string image_compressed = image_topic + "/thread/compressed";
        publisher_vector[i] = nh.advertise<sensor_msgs::CompressedImage>(image_compressed, 1);
        publish_seq[i] = 0;
        std::cerr << "Topics: " << image_topic << std::endl;
        subscriber_vector[i] = nh.subscribe<sensor_msgs::Image>(image_topic, 100, boost::bind(&imageCallback, _1, i));
    }
    for (int tid = 0; tid < num_threads; tid++)
    {
        vec_thread[tid] = std::thread(compressor_thread_func, tid);
    }

    ros::spin();
    }

#pragma clang diagnostic pop