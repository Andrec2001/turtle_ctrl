#include "rclcpp/rclcpp.hpp"
#include "turtlesim_msgs/msg/pose.hpp"

using std::placeholders::_1;

//topic dal quale leggere la posizione
//tipo msg posizione: turtlesim_msgs/msg/Pose

class TurtleCtrl : public rclcpp::Node 
{
    //Subscriber al topic
    rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr poseSubscriber_;
    
    //variabile in cui salviamo la posizione letta dal Subscriber
    turtlesim_msgs::msg::Pose poseMsg_;

public: 
    //Costruttore della classe

    TurtleCtrl() : Node("turtle_ctrl_node")
    {
        poseSubscriber_ = this->create_subscription<turtlesim_msgs::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleCtrl::PoseCallback,this, _1));
    }
    void PoseCallback(const turtlesim_msgs::msg::Pose::SharedPtr msg)
    {
        //Stampa la posizione di turtle1 
        poseMsg_ = *msg;
        std::cout <<"Pose (x,y,θ): " << poseMsg_.x << ", " << poseMsg_.y << ", " << poseMsg_.theta << std::endl; 
    }

};


int main(int argc,char *argv[])
{
    //Inizializzo ROS
    rclcpp::init(argc, argv);

    //Definisco e lancio il nodo
    std::shared_ptr ctrlNode = std::make_shared<TurtleCtrl>();
    rclcpp::spin(ctrlNode);

    //Al termine (Ctrl+C) chiudo ROS
    rclcpp::shutdown();
    return 0;
}
