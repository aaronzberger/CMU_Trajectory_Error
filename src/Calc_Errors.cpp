#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

unsigned closest(const std::vector<std::vector<double>> &vec, double val);
unsigned findValid(const std::vector<std::vector<double>> &vec, unsigned index);

int main(int argc, char **argv) {
    //Open the CSV file
    std::ifstream file("src/trajectory_error/field_files/global_path_1.csv");
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
    bag.open("src/trajectory_error/field_files/nav_test_1.bag");
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

    unsigned bagSize {viewer.size()};

    double positionErrorTotal{0};
    double orientationErrorTotal{0};
    int totalEntries{0};

    //Loop through csv Vector and find corresponding bag entries. Calculate error and add to error Vector
    for (auto csvMsg : csvVec) {
        unsigned bagMsgIndex{closest(bagVec, csvMsg.at(0))};        
        if (bagVec.at(bagMsgIndex).at(0) == csvMsg.at(0)) {
            double positionError{pow((csvMsg.at(1) - bagVec.at(bagMsgIndex).at(1)), 2) + pow((csvMsg.at(2) - bagVec.at(bagMsgIndex).at(2)), 2)};
            double orientationError{abs(csvMsg.at(3) - bagVec.at(bagMsgIndex).at(3))};

            std::vector<double> instance;

            instance.push_back(bagVec.at(bagMsgIndex).at(0));
            instance.push_back(positionError);
            instance.push_back(orientationError);

            errorVec.push_back(instance);

            positionErrorTotal += positionError;
            orientationErrorTotal += orientationError;
            totalEntries++;
        }
    }

    //Print means and vectors
    for (auto vec : errorVec) {
        std::cout << boost::format("Time: [%.15f], Position Error: [%.15f], Orientation Error: [%.15f]") 
            % vec.at(0) % vec.at(1) % vec.at(2) << std::endl;
    }
    std::cout << boost::format("Position Error Mean: [%.5f]") % (positionErrorTotal / totalEntries) << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % (orientationErrorTotal / totalEntries) << std::endl;
    std::cout << boost::format("Total Entries: %d, %d entries were not in the bag file")
        % totalEntries % (csvVec.size() - 1 - totalEntries) << std::endl;
}

unsigned closest(const std::vector<std::vector<double>> &vec, double val) {
    if (val <= vec.at(0).at(0))
        return findValid(vec, 0);
    if (val >= vec.at(vec.size() - 1).at(0))
        return findValid(vec, vec.size() - 1);

    unsigned i = 0, j = vec.size() - 1, mid = 0;
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

unsigned findValid(const std::vector<std::vector<double>> &vec, unsigned index) {
    if(!isnan(vec.at(index).at(3))) {
        return index;
    }
    if(index >= 5 && index <= vec.size() - 6) {
        for(unsigned i{index - 5}; i < index + 5; i++) {
            if(i != index && (!isnan(vec.at(i).at(3))) && (vec.at(index).at(0) == vec.at(i).at(0))) {
                return i;
            }
        }
    }
    return index;
}