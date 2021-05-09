#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <line_detection.h>
#include <line_detection_types.h>

namespace line_detector {
    
    class LineDetectorNodelet : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber img_sub;
        ros::Publisher data_pub;
        ros::Publisher img_pub;
        
        // Declare the output data (defined in codegen/lib/line_detect/examples/main.cu)
        unsigned char Iout_data[2764800];
        float predictedPosNorm[2];
        int I_size[3];
        int Iout_size[3];
        
    public:
        virtual void onInit()
        {
            NODELET_INFO("Loading network model...");
            
            // Initialization
            cv::Mat in_image = cv::Mat::zeros(cv::Size(320,180), CV_8UC3);
            I_size[0] = 3;
            I_size[1] = 320;
            I_size[2] = 180;
            
            // Call the entry-point to preload the network
            line_detection(in_image.data, I_size, predictedPosNorm, Iout_data, Iout_size);
            
            NODELET_INFO("Node starting...");
            nh_ = getNodeHandle();
            std::string data_topic_name = "detection_pos";
            std::string img_topic_name = "detection_image";
            img_sub = nh_.subscribe("/image_raw", 1, &LineDetectorNodelet::msgCallback, this);
            data_pub = nh_.advertise<std_msgs::Float32MultiArray>(data_topic_name, 1);
            img_pub = nh_.advertise<sensor_msgs::Image>(img_topic_name, 1);
            NODELET_INFO("Node started successfully");
        }
        
        //Callback for new image message data
        void msgCallback(const sensor_msgs::Image::ConstPtr& inmsg)
        {
            //Declare the input data
            cv::Mat in_image = cv_bridge::toCvShare(inmsg, sensor_msgs::image_encodings::BGR8)->image;
            int width = in_image.cols;
            int height = in_image.rows;
            I_size[0] = 3;
            I_size[1] = width;
            I_size[2] = height;
            
            // Call the entry-point 'line_detection'.
            line_detection(in_image.data, I_size, predictedPosNorm, Iout_data, Iout_size);
            
            NODELET_INFO("Predicted point:(%3.2f,%3.2f)",predictedPosNorm[0],predictedPosNorm[1]);
            
            // Declare output message and substitute
            std_msgs::Float32MultiArray dataoutmsg;
            dataoutmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            dataoutmsg.layout.dim[0].size = 1;
            dataoutmsg.layout.dim[0].stride = 2;
            dataoutmsg.layout.dim[0].label = "Predicted point in line";
            dataoutmsg.data.clear();
            dataoutmsg.data.push_back(predictedPosNorm[0]);
            dataoutmsg.data.push_back(predictedPosNorm[1]);
            
            //Publish the detected data
            data_pub.publish(dataoutmsg);
            
            //Publish the annonated image
            cv::Mat imgout(Iout_size[2], Iout_size[1], CV_8UC3, Iout_data);
            sensor_msgs::Image::Ptr imgoutmsg = cv_bridge::CvImage(inmsg->header, sensor_msgs::image_encodings::BGR8, imgout).toImageMsg();
            img_pub.publish(imgoutmsg);
            
        }
    };
    
    
}
PLUGINLIB_EXPORT_CLASS(line_detector::LineDetectorNodelet, nodelet::Nodelet);
