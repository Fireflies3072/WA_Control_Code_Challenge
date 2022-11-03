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

class MergeArrays : public rclcpp::Node
{
public:
	MergeArrays()
	: Node("merge_arrays")
	{
		input1 = this->create_subscription<std_msgs::msg::Int32MultiArray>("input1", 10, std::bind(&MergeArrays::Input1Callback, this, _1));
		input2 = this->create_subscription<std_msgs::msg::Int32MultiArray>("input2", 10, std::bind(&MergeArrays::Input2Callback, this, _1));
		output = this->create_publisher<std_msgs::msg::Int32MultiArray>("output", 20);
		Merge();
	}

private:
	vector<int> array1;
	vector<int> array2;
	vector<int> array;

	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr input1;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr input2;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output;

	void Input1Callback(const std_msgs::msg::Int32MultiArray::SharedPtr arr)
	{
		array1.clear();
		string strInput1 = "array1 Received:";
		for (int i=0; i<arr->data.size(); i++)
		{
			array1.push_back(arr->data[i]);
			strInput1 += " ";
			strInput1 += to_string(array1[i]);
		}
		RCLCPP_INFO(this->get_logger(), strInput1);
		Merge();
	}

	void Input2Callback(const std_msgs::msg::Int32MultiArray::SharedPtr arr)
	{
		array2.clear();
		string strInput2 = "array2 Received:";
		for (int i=0; i<arr->data.size(); i++)
		{
			array2.push_back(arr->data[i]);
			strInput2 += " ";
			strInput2 += to_string(array2[i]);
		}
		RCLCPP_INFO(this->get_logger(), strInput2);
		Merge();
	}
	
	void Merge()
	{
		if (array1.size() == 0 || array2.size() == 0)
		{
			return;
		}
		
		// merge array
		array.clear();
		int j = 0;
		for (int i=0; i<array1.size(); i++)
		{
			while (j<array2.size() && array1[i]>array2[j])
			{
				array.push_back(array2[j]);
				j++;
			}
			array.push_back(array1[i]);
		}
		while (j < array2.size())
		{
			array.push_back(array2[j]);
			j++;
		}
		
		// output
		auto outputArray = std_msgs::msg::Int32MultiArray();
		outputArray.data.clear();
		string strOutput = "array Published:";
		for (int i=0; i<array.size(); i++)
		{
			outputArray.data.push_back(array[i]);
			strOutput += " ";
			strOutput += to_string(array[i]);
		}
		RCLCPP_INFO(this->get_logger(), strOutput);
		output->publish(outputArray);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MergeArrays>());
	rclcpp::shutdown();
	return 0;
}
