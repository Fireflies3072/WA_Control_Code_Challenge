// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class InputArrays : public rclcpp::Node
{
public:
	InputArrays()
	: Node("input_arrays")
	{
		output1 = this->create_publisher<std_msgs::msg::Int32MultiArray>("input1", 10);
		output2 = this->create_publisher<std_msgs::msg::Int32MultiArray>("input2", 10);
		timer = this->create_wall_timer(500ms, std::bind(&InputArrays::PublishInput, this));
	}

private:
	vector<int> array1;
	vector<int> array2;

	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output1;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output2;
	rclcpp::TimerBase::SharedPtr timer;
	
	void PublishInput()
	{
		array1 = {1, 4, 8, 12, 26};
		array2 = {3, 9, 18, 20, 30};

		auto outputArray1 = std_msgs::msg::Int32MultiArray();
		auto outputArray2 = std_msgs::msg::Int32MultiArray();
		outputArray1.data.clear();
		outputArray2.data.clear();
		
		// publish array1
		string strOutput1 = "array1 Published:";
		for (int i=0; i<array1.size(); i++)
		{
			outputArray1.data.push_back(array1[i]);
			strOutput1 += " ";
			strOutput1 += to_string(array1[i]);
		}
		output1->publish(outputArray1);
		RCLCPP_INFO(this->get_logger(), strOutput1);
		
		// publish array2
		string strOutput2 = "array2 Published:";
		for (int i=0; i<array2.size(); i++)
		{
			outputArray2.data.push_back(array2[i]);
			strOutput2 += " ";
			strOutput2 += to_string(array2[i]);
		}
		output2->publish(outputArray2);
		RCLCPP_INFO(this->get_logger(), strOutput2);
		timer->cancel();
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<InputArrays>());
	rclcpp::shutdown();
	return 0;
}
