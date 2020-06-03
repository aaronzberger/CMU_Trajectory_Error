#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

int closest(const std::vector<std::vector<double>> &vec, double val);
int findValid(const std::vector<std::vector<double>> &vec, int index);

int main(int argc, char **argv) {
    //Open the CSV file
    std::ifstream file("/home/aaron/Documents/traj_bags/global_path_1.csv");
    if (!file.is_open()) {
        std::cout << "Could not open CSV File" << std::endl;
        return 1;
    }

    //Declare vectors for the bag and csv files, and the calculated error
    std::vector<std::vector<double>> csvVec;
    std::vector<std::vector<double>> bagVec;
    std::vector<std::vector<double>> errorVec;

    std::string secs, nsecs, x, y, yaw;

    //Parse out the first line, which are the titles of the columns
    std::getline(file, secs, '\n');

    //loop through CSV file and copy data to a vector. Calculate the time while copying
    int row {0};
    while (file.good() && row < 3339) {
        std::getline(file, secs, ',');
        std::getline(file, nsecs, ',');
        std::getline(file, x, ',');
        std::getline(file, y, ',');
        std::getline(file, yaw, '\n');

        std::vector<double> instance;

        instance.push_back(std::stod(secs) + (std::stod(nsecs) * (pow(10, -9))));
        instance.push_back(std::stod(x));
        instance.push_back(std::stod(y));
        instance.push_back(std::stod(yaw));

        csvVec.push_back(instance);
        row++;
    }
    file.close();

    //Open the .bag file
    rosbag::Bag bag;
    bag.open("/home/aaron/Documents/traj_bags/nav_test_1.bag");
    rosbag::View viewer(bag);

    //Loop through the bag file and copy some contents to a vector
    for (const rosbag::MessageInstance msg : viewer) {
        nav_msgs::Odometry::ConstPtr thisMsg = msg.instantiate<nav_msgs::Odometry>();
        if (thisMsg != nullptr) {
            //Convert Quaternion orientation to Euler
            tf::Quaternion quaternion;
            quaternion.setW(thisMsg->pose.pose.orientation.w);
            quaternion.setX(thisMsg->pose.pose.orientation.x);
            quaternion.setY(thisMsg->pose.pose.orientation.y);
            quaternion.setZ(thisMsg->pose.pose.orientation.z);

            double yaw {tf::getYaw(quaternion)};
            std::vector<double> instance;

            instance.push_back(thisMsg->header.stamp.sec + (thisMsg->header.stamp.nsec * (pow(10, -9))));
            instance.push_back(thisMsg->pose.pose.position.x);
            instance.push_back(thisMsg->pose.pose.position.y);
            instance.push_back(yaw);

            bagVec.push_back(instance);
        }
        else {
            std::cout << "Could not retrieve this message" << std::endl;
        }
    }
    bag.close();

    int bagSize {viewer.size()};

    double positionErrorTotal{0};
    double orientationErrorTotal{0};
    int totalEntries{0};

    //Loop through csv Vector and find corresponding bag entries. Calculate error and add to error Vector
    for (auto csvMsg : csvVec) {
        double bagMsg{closest(bagVec, csvMsg.at(0))};        
        if (bagVec.at(bagMsg).at(0) == csvMsg.at(0)) {
            double positionError{pow((csvMsg.at(1) - bagVec.at(bagMsg).at(1)), 2) + pow((csvMsg.at(2) - bagVec.at(bagMsg).at(2)), 2)};
            double orientationError{abs(csvMsg.at(3) - bagVec.at(bagMsg).at(3))};

            std::vector<double> instance;

            instance.push_back(bagVec.at(bagMsg).at(0));
            instance.push_back(positionError);
            instance.push_back(orientationError);

            errorVec.push_back(instance);

            positionErrorTotal += positionError;
            orientationErrorTotal += orientationError;
            totalEntries++;
        }
    }

    //Print means and vectors
    std::cout.precision(20);
    for (auto vec : errorVec) {
        std::cout << "Time: [" << vec.at(0) << "], Pos Error: [" << vec.at(1) << "], Orient Error: [" << vec.at(2) << "]" << std::endl;
    }
    
    std::cout << "Position Error Mean: [" << positionErrorTotal / totalEntries << "]" << std::endl;
    std::cout << "Position Error Mean: [" << orientationErrorTotal / totalEntries << "]" << std::endl;
    std::cout << "Total Entries: " << totalEntries << ", " << 
        csvVec.size() - 1 - totalEntries << " entries were not found" <<std::endl;
}

int closest(const std::vector<std::vector<double>> &vec, double val) {
    if (val <= vec.at(0).at(0))
        return findValid(vec, 0);
    if (val >= vec.at(vec.size() - 1).at(0))
        return findValid(vec, vec.size() - 1);

    int i = 0, j = vec.size() - 1, mid = 0;
    while (i < j) {
        mid = (i + j) / 2;
        if (vec.at(mid).at(0) == val)
            return findValid(vec, mid);
        if (vec.at(mid).at(0) > val) {
            j = mid;
        }
        else {
            i = mid + 1;
        }
    }
    return findValid(vec, mid);
}

int findValid(const std::vector<std::vector<double>> &vec, int index) {
    while(isnan(vec.at(index).at(3))) {
        index++;
    }
    return index;
}