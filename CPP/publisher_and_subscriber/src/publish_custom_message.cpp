/**
 * @file publisher_custom_message.cpp
 *
 * @brief A basic publisher class node which use a custom made message
 *        interface defined within the same package so it doesn't need
 *        Custom msg and srv to compile
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "publisher_and_subscriber/msg/employee_salary.hpp"
#include <string>
#include <vector>


using namespace std::chrono_literals;


class MyCustomMsgPublisher : public rclcpp::Node {
private:
  rclcpp::Publisher<publisher_and_subscriber::msg::EmployeeSalary>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void TimerCallback();
  std::string name_;
  std::string surname_;
  bool gender_;
  std::vector<std::string> tasks_;
  int salary_;
  publisher_and_subscriber::msg::EmployeeSalary message_;

public:
  MyCustomMsgPublisher(std::vector<std::string> passedTasks,
                       const std::string passedNodeName = "VOID",
                       const std::string passedName = "VOID",
                       const std::string passedSurname = "VOID",
                       int passedSalary = 0)
    : Node(passedNodeName), name_(passedName), 
      surname_(passedSurname), salary_(passedSalary)
  {
    pub_ = this->create_publisher<publisher_and_subscriber::msg::EmployeeSalary>(
      "/employee_salary", 10);
    message_.name = name_;
    message_.surname = surname_;
    for(size_t i=0; i<passedTasks.size(); i++)
    {
      message_.tasks[i] = passedTasks[i];
    }
    message_.salary = salary_;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyCustomMsgPublisher::TimerCallback, this));
  }
};

void MyCustomMsgPublisher::TimerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Publishing to the topic");
  pub_->publish(message_); 
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::string name = "";
  std::string surname = "";
  int numTasks{0};
  std::vector<std::string> tasks{};
  int salary{0};

  std::cout << "Insert the name -> ";
  std::cin >> name;
  std::cout << "Insert the surname -> ";
  std::cin >> surname;
  std::cout << "How many tasks for the employee (up to 3) -> ";
  std::cin >> numTasks;
  std::string task;
  for (int i = 0; i < numTasks; i++) {
    std::cout << "Insert the task number " + std::to_string(i + 1) + " -> ";
    std::cin >> task;
    tasks.push_back(task);
  }
  std::cout << "Insert the salary -> ";
  std::cin >> salary;
  auto node = std::make_shared<MyCustomMsgPublisher>(
      tasks, "custom_msg_publisher", name, surname, salary);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
