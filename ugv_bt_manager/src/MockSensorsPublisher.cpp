// Copyright (c) 2025 Matheus França
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <map>
#include <string>

/**
 * @class MockSensorsPublisher
 * @brief A ROS node to publish simulated sensor data for testing purposes.
 *
 * This class creates a ROS node that publishes data to various sensor topics.
 * It reads a "test_scenario" parameter to determine which test case to execute.
 * Scenarios include publishing all-safe data, or simulating a temporary fault
 * in one of the sensors (e-stop, battery, temperature, etc.).
 */
class MockSensorsPublisher
{
public:
    /**
     * @brief Constructor for the MockSensorsPublisher class.
     * @param nh A ROS NodeHandle.
     */
    MockSensorsPublisher(ros::NodeHandle& private_nh)
    {
        // Get the test scenario from the parameter server, default to 1 (all safe)
        private_nh.param<int>("test_scenario", test_scenario_, 1);

        // Initialize publishers for all sensor topics
        ros::NodeHandle nh_;
        pubs_["estop"] = nh_.advertise<std_msgs::Bool>("estop", 1, true);
        pubs_["internet_signal"] = nh_.advertise<std_msgs::Int32>("internet_signal", 1, true);
        pubs_["battery_level"] = nh_.advertise<std_msgs::Float32>("battery_level", 1, true);
        pubs_["temperature"] = nh_.advertise<std_msgs::Float32>("temperature", 1, true);
        pubs_["gps_accuracy"] = nh_.advertise<std_msgs::Float32>("gps_accuracy", 1, true);
        pubs_["crosstrack_err"] = nh_.advertise<std_msgs::Float32>("crosstrack_err_mock", 1, true);

        ROS_INFO("Sensor Test Publisher started. Executing Test Scenario: %d", test_scenario_);
    }

    /**
     * @brief Runs the main loop of the publisher based on the selected scenario.
     */
    void run()
    {
        ros::Rate loop_rate(1); // Loop at 1 Hz

        while (ros::ok())
        {
            switch (test_scenario_)
            {
                case 1:
                    runAllSafeScenario();
                    break;
                case 2:
                    runEstopFaultScenario();
                    break;
                case 3:
                    runLowBatteryScenario();
                    break;
                case 4:
                    runHighTempScenario();
                    break;
                case 5:
                    runBadGpsScenario();
                    break;
                case 6:
                    runHighCrosstrackScenario();
                    break;
                case 7:
                    runNoInternetScenario();
                    break;
                case 8:
                    runLowInternetScenario();
                    break;
                default:
                    ROS_WARN("Unknown test_scenario [%d]. Defaulting to All Safe.", test_scenario_);
                    runAllSafeScenario();
                    break;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> pubs_;
    int test_scenario_;

    // Define safe state values
    const bool SAFE_ESTOP = false;
    const int SAFE_INTERNET = 1;
    const float SAFE_BATTERY = 85.0;
    const float SAFE_TEMP = 40.0;
    const float SAFE_GPS = 300.0;
    const float SAFE_CROSSTRACK = 0.1;

    /**
     * @brief Publishes a set of sensor values to their respective topics.
     */
    void publish_state(bool estop, int internet, float battery, float temp, float gps, float crosstrack)
    {
        std_msgs::Bool b; b.data = estop;
        pubs_["estop"].publish(b);

        std_msgs::Int32 i; i.data = internet;
        pubs_["internet_signal"].publish(i);

        std_msgs::Float32 f;
        f.data = battery; pubs_["battery_level"].publish(f);
        f.data = temp; pubs_["temperature"].publish(f);
        f.data = gps; pubs_["gps_accuracy"].publish(f);
        f.data = crosstrack; pubs_["crosstrack_err"].publish(f);

        ROS_INFO("Published State: [E-Stop: %d, Internet: %d, Battery: %.1f, Temp: %.1f, GPS: %.1f, Crosstrack: %.2f]",
                 estop, internet, battery, temp, gps, crosstrack);
    }

    /**
     * @brief Scenario 1: Continuously publish safe data.
     */
    void runAllSafeScenario() {
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
    }

    /**
     * @brief Scenario 2: Simulate a temporary E-Stop fault.
     */
    void runEstopFaultScenario() {
        ROS_INFO("[SCENARIO] Simulating E-Stop fault...");
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep(); // 5 seconds of safe state

        publish_state(true, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(5.0).sleep(); // 5 seconds of unsafe state

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep(); // 5 seconds of safe state
    }

    /**
     * @brief Scenario 3: Simulate a temporary low battery fault.
     */
    void runLowBatteryScenario() {
        ROS_INFO("[SCENARIO] Simulating low battery fault...");
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, 45.0, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }

    /**
     * @brief Scenario 4: Simulate a temporary high temperature fault.
     */
    void runHighTempScenario() {
        ROS_INFO("[SCENARIO] Simulating high temperature fault...");
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, 60.0, SAFE_GPS, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }
    
    /**
     * @brief Scenario 5: Simulate a temporary bad GPS fault.
     */
    void runBadGpsScenario() {
        ROS_INFO("[SCENARIO] Simulating bad GPS fault...");

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, 150.0, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(20.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }

    /**
     * @brief Scenario 6: Simulate a temporary high crosstrack error fault.
     */
    void runHighCrosstrackScenario() {
        ROS_INFO("[SCENARIO] Simulating high crosstrack error fault...");
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, 1.8); // Unsafe state
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }

    /**
     * @brief Scenario 7: Simulate a temporary internet disconnection.
     */
    void runNoInternetScenario() {
        ROS_INFO("[SCENARIO] Simulating no internet fault...");
        publish_state(SAFE_ESTOP, 0, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(15.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }

    /**
     * @brief Scenario 8: Simulate a temporary internet low signal.
     */
    void runLowInternetScenario() {
        ROS_INFO("[SCENARIO] Simulating low internet signal...");
        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();

        publish_state(SAFE_ESTOP, 2, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK); // Unsafe state
        ros::Duration(25.0).sleep();

        publish_state(SAFE_ESTOP, SAFE_INTERNET, SAFE_BATTERY, SAFE_TEMP, SAFE_GPS, SAFE_CROSSTRACK);
        ros::Duration(5.0).sleep();
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mock_sensors_publisher");
    ros::NodeHandle private_nh("~");

    MockSensorsPublisher publisher(private_nh);
    publisher.run();

    return 0;
}
