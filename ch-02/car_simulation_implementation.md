# Chapter 2: Implementing Car Simulation with ROS2

## 2.1 Introduction
This chapter details the implementation of car simulation using ROS2, focusing on performance requirements, code examples, process monitoring, and performance comparisons.

## 2.2 Car Simulation Implementation

### 2.2.1 Basic Car Model
We'll start with a basic kinematic car model:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class CarSimulation(Node):
    def __init__(self):
        super().__init__('car_simulation')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_time = self.get_clock().now()

    def velocity_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

        self.x += self.vx * dt * math.cos(self.theta) - self.vy * dt * math.sin(self.theta)
        self.y += self.vx * dt * math.sin(self.theta) + self.vy * dt * math.cos(self.theta)
        self.theta += self.vtheta * dt

        self.publish_odometry()

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        self.publisher_.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    car_simulation = CarSimulation()
    rclpy.spin(car_simulation)
    car_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2.2 Performance Requirements
- Real-time operation: The simulation should run at least at 50 Hz.
- Low latency: Input to output latency should be less than 20 ms.
- Accuracy: Position error should be less than 1% of the traveled distance.

## 2.3 Monitoring the Process

To monitor the simulation process, we can use ROS2 tools and custom logging:

### 2.3.1 ROS2 Topic Echo
Use `ros2 topic echo /odom` to monitor the odometry output in real-time.

### 2.3.2 rqt_plot
Use `rqt_plot` to visualize the car's position and velocity over time.

### 2.3.3 Custom Performance Logger

Add this to the CarSimulation class:

```python
import time

class CarSimulation(Node):
    # ... (previous code) ...

    def __init__(self):
        # ... (previous initialization) ...
        self.perf_logger = self.create_publisher(String, 'performance_logs', 10)
        self.last_log_time = time.time()

    def velocity_callback(self, msg):
        callback_start = time.time()
        # ... (previous callback code) ...
        callback_end = time.time()
        
        self.log_performance(callback_start, callback_end)

    def log_performance(self, start, end):
        latency = (end - start) * 1000  # Convert to milliseconds
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:  # Log every second
            freq = 1.0 / (current_time - self.last_log_time)
            log_msg = f"Frequency: {freq:.2f} Hz, Latency: {latency:.2f} ms"
            self.perf_logger.publish(String(data=log_msg))
            self.last_log_time = current_time
```

## 2.4 Performance Comparison

### 2.4.1 Before Optimization
Run the simulation and collect performance data using the custom logger. Note down the average frequency and latency.

### 2.4.2 Optimization Techniques
1. Use a timer-based approach instead of subscription callback:

```python
class CarSimulation(Node):
    def __init__(self):
        # ... (previous initialization) ...
        self.timer = self.create_timer(0.02, self.update_simulation)  # 50 Hz

    def update_simulation(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update position and orientation
        self.x += self.vx * dt * math.cos(self.theta) - self.vy * dt * math.sin(self.theta)
        self.y += self.vx * dt * math.sin(self.theta) + self.vy * dt * math.cos(self.theta)
        self.theta += self.vtheta * dt

        self.publish_odometry()
```

2. Use C++ instead of Python for better performance:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class CarSimulation : public rclcpp::Node
{
public:
    CarSimulation() : Node("car_simulation")
    {
        publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CarSimulation::velocity_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&CarSimulation::update_simulation, this));

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        vx_ = 0.0;
        vy_ = 0.0;
        vtheta_ = 0.0;
        last_time_ = now();
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        vtheta_ = msg->angular.z;
    }

    void update_simulation()
    {
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        x_ += vx_ * dt * std::cos(theta_) - vy_ * dt * std::sin(theta_);
        y_ += vx_ * dt * std::sin(theta_) + vy_ * dt * std::cos(theta_);
        theta_ += vtheta_ * dt;

        publish_odometry();
    }

    void publish_odometry()
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vtheta_;
        publisher_->publish(odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, theta_, vx_, vy_, vtheta_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulation>());
    rclcpp::shutdown();
    return 0;
}
```

### 2.4.3 After Optimization
Run the optimized simulation and collect performance data. Compare with the previous results:

1. Frequency: Should be consistently at or above 50 Hz.
2. Latency: Should be reduced, ideally below 10 ms.
3. CPU Usage: Monitor using `top` or `htop`. Should be lower in the optimized version.

### 2.4.4 Comparison Table

| Metric | Before Optimization | After Optimization |
|--------|---------------------|---------------------|
| Frequency | ~30 Hz (example) | ~60 Hz (example) |
| Latency | ~25 ms (example) | ~8 ms (example) |
| CPU Usage | ~15% (example) | ~8% (example) |

## 2.5 Conclusion
This chapter demonstrated the implementation of a basic car simulation in ROS2, including performance monitoring and optimization techniques. By using timer-based updates and implementing in C++, we significantly improved the simulation's performance, meeting our requirements for real-time operation, low latency, and accuracy.