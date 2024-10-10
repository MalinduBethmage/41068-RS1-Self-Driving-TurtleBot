#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanMatcherNode : public rclcpp::Node {
public:
    ScanMatcherNode() : Node("scan_matcher_node") {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatcherNode::scanCallback, this, std::placeholders::_1));
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ScanMatcherNode::mapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Scan Matcher Node started.");
    }

private:
    // Store map as a cv::Mat image
    cv::Mat map_image_, edge_image_;

    // Callback for receiving the map (Image A)
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        map_image_ = cv::Mat(height, width, CV_8UC1);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = x + y * width;
                int value = msg->data[index];
                if (value == -1) {
                    map_image_.at<uchar>(y, x) = 127; // Unknown area
                } else {
                    map_image_.at<uchar>(y, x) = (value == 100) ? 0 : 255; // Occupied or free space
                }
            }
        }

        // Show Image A (Map)
        cv::imshow("Image A - Map", map_image_);
        cv::waitKey(1);

        // Extract edges (Image B)
        cv::Canny(map_image_, edge_image_, 50, 150);
        cv::imshow("Image B - Edge Map", edge_image_);
        cv::waitKey(1);
    }

    // Callback for receiving laser scans (Image C)
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        cv::Mat laser_image = laserScanToMat(msg);
        if (!laser_image.empty()) {
            // Show Image C (Laser Scan Image)
            cv::imshow("Image C - Laser Scan", laser_image);
            cv::waitKey(1);
        }
    }

    // Convert laser scan to a cv::Mat image (Image C)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;  // Set pixel to white for detected points
                }
            }
        }

        return image;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatcherNode>());
    rclcpp::shutdown();
    return 0;
}
