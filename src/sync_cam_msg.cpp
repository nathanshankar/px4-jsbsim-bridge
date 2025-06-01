#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;

class StereoDisparityNode : public rclcpp::Node {
public:
    StereoDisparityNode() : Node("stereo_disparity_node") {
        // Subscribers to left and right images
        left_sub_.subscribe(this, "/fgfs/left/image_raw");
        right_sub_.subscribe(this, "/fgfs/right/image_raw");

        // Approximate time sync for stereo image messages
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&StereoDisparityNode::image_callback, this, _1, _2));

        // Publisher for colorized disparity image
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fgfs/disparity/color", 10);

        // StereoBM parameters
        stereo_bm_ = cv::StereoBM::create(64, 15);  // numDisparities, blockSize
        RCLCPP_INFO(this->get_logger(), "Stereo Disparity Node initialized");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {
        try {
            // Convert to grayscale OpenCV images
            auto left_cv = cv_bridge::toCvCopy(left_msg, "mono8");
            auto right_cv = cv_bridge::toCvCopy(right_msg, "mono8");

            cv::Mat disparity_raw, disparity_normalized, disparity_color;

            // Compute raw disparity
            stereo_bm_->compute(left_cv->image, right_cv->image, disparity_raw);

            // Normalize disparity to 0-255 range
            disparity_raw.convertTo(disparity_normalized, CV_8U, 255.0 / (stereo_bm_->getNumDisparities() * 16.0));

            // Apply a color map (e.g. Jet) to create an RGB image
            cv::applyColorMap(disparity_normalized, disparity_color, cv::COLORMAP_JET);

            // Publish as BGR8 image
            auto disp_msg = cv_bridge::CvImage(left_msg->header, "bgr8", disparity_color).toImageMsg();
            disparity_pub_->publish(*disp_msg);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // Message filter subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_, right_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Publisher for RGB disparity image
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;

    // OpenCV StereoBM matcher
    cv::Ptr<cv::StereoBM> stereo_bm_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoDisparityNode>());
    rclcpp::shutdown();
    return 0;
}
