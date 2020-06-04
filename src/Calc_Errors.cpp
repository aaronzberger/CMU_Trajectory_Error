#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <boost/format.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

unsigned closest(const std::vector<std::vector<double>> &vec, double val);
unsigned findValid(const std::vector<std::vector<double>> &vec, unsigned index);

const unsigned timeIndex = 0;
const unsigned xIndex = 1;
const unsigned yIndex = 2;
const unsigned yawIndex = 3;

const unsigned positionErrorIndex = 1;
const unsigned orientationErrorIndex = 2;

const double zScoreSignificanceLevel = 3;
const unsigned int msgSearchRadius = 5;

const unsigned outlierAtPosition = 0;
const unsigned outlierAtOrientation = 1;
const unsigned outlierAtBoth = 2;

const unsigned indexInErrorVecIndex = 0;
const unsigned outlierIdentifierIndex = 1;

int main(int argc, char **argv) {

    if(argc != 3) {
        perror("Expected two arguments: CSV file, BAG file\n");
        return 1;
    }

    //Open the CSV file
    std::ifstream file(argv[1]);
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
    while (file.good()) {
        std::getline(file, secs, ',');
        std::getline(file, nsecs, ',');
        std::getline(file, x, ',');
        std::getline(file, y, ',');
        std::getline(file, yaw, '\n');

        if(file.eof()) break;

        std::vector<double> instance;

        instance.push_back(std::stod(secs) + (std::stod(nsecs) * (pow(10, -9))));
        instance.push_back(std::stod(x));
        instance.push_back(std::stod(y));
        instance.push_back(std::stod(yaw));

        csvVec.push_back(instance);
    }
    file.close();

    //Open the .bag file
    rosbag::Bag bag;
    bag.open(argv[2]);
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
        unsigned bagMsgIndex{closest(bagVec, csvMsg.at(timeIndex))};       
        if (bagVec.at(bagMsgIndex).at(timeIndex) == csvMsg.at(timeIndex)) {
            double positionError{pow((csvMsg.at(xIndex) - bagVec.at(bagMsgIndex).at(xIndex)), 2) + 
                                 pow((csvMsg.at(yIndex) - bagVec.at(bagMsgIndex).at(yIndex)), 2)};
            double orientationError{abs(csvMsg.at(yawIndex) - bagVec.at(bagMsgIndex).at(yawIndex))};

            std::vector<double> instance;

            instance.push_back(bagVec.at(bagMsgIndex).at(timeIndex));
            instance.push_back(positionError);
            instance.push_back(orientationError);

            errorVec.push_back(instance);

            positionErrorTotal += positionError;
            orientationErrorTotal += orientationError;
            totalEntries++;
        }
    }

    double positionErrorMean {positionErrorTotal / totalEntries};
    double orientationErrorMean {orientationErrorTotal / totalEntries};

    double positionErrorSum {0};
    double orientationErrorSum {0};
    for(int i {0}; i < errorVec.size(); i++) {
        positionErrorSum += pow(errorVec.at(i).at(positionErrorIndex), 2);
        orientationErrorSum += pow(errorVec.at(i).at(orientationErrorIndex), 2);
    }
    positionErrorSum /= errorVec.size() - 1;
    orientationErrorSum /= errorVec.size() - 1;
    double positionErrorSD {sqrt(positionErrorSum)};
    double orientationErrorSD {sqrt(orientationErrorSum)};

    //Construst a vector to store outliers
    //The first int is index in the errorVec, the second identifies which value is the outler
    //0:position, 1:orientation, 2:both
    std::vector<std::vector<int>> outliers;

    double positionErrorMeanOutliers {0};
    int positionErrorMeanCount {0};
    double orientationErrorMeanOutliers {0};
    int orientationErrorMeanCount {0};

    for(int i {0}; i < errorVec.size(); i++) {
        double positionErrorZScore {(errorVec.at(i).at(positionErrorIndex) - positionErrorMean) / positionErrorSD};
        double orientationErrorZScore {(errorVec.at(i).at(orientationErrorIndex) - orientationErrorMean) / orientationErrorSD};

        if(abs(positionErrorZScore) > zScoreSignificanceLevel || abs(orientationErrorZScore) > zScoreSignificanceLevel) {
            std::vector<int> instance;
            instance.push_back(i);
            if(abs(positionErrorZScore) > zScoreSignificanceLevel && abs(orientationErrorZScore) > zScoreSignificanceLevel) {
                instance.push_back(outlierAtBoth);
            } else if(abs(positionErrorZScore) > zScoreSignificanceLevel) {
                instance.push_back(outlierAtPosition);
                orientationErrorMeanCount++;
                orientationErrorMeanOutliers += errorVec.at(i).at(orientationErrorIndex);
            } else {
                instance.push_back(outlierAtOrientation);
                positionErrorMeanCount++;
                positionErrorMeanOutliers += errorVec.at(i).at(positionErrorIndex);
            }
            outliers.push_back(instance);
        } else {
            positionErrorMeanCount++;
            orientationErrorMeanCount++;
            positionErrorMeanOutliers += errorVec.at(i).at(positionErrorIndex);
            orientationErrorMeanOutliers += errorVec.at(i).at(orientationErrorIndex);
        }
    }

    positionErrorMeanOutliers /= positionErrorMeanCount;
    orientationErrorMeanOutliers /= orientationErrorMeanCount;

    //Print data
    //Individual Entries
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "INDIVIDUAL ENTRIES\n" << std::endl;
    for (auto vec : errorVec) {
        std::cout << boost::format("Time: [%.3f], Position Error: [%.5f], Orientation Error: [%.5f]") 
            % vec.at(timeIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }
    //Outliers
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "OUTLIERS\n" << std::endl;

    for(auto vec: outliers) {
        std::cout << boost::format("Stamp: [%.3f], ") % errorVec.at(vec.at(0)).at(timeIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format("Position Error: [%.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format("Orientation Error: [%.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        std::cout << std::endl;
    }
    //Analysis
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "ANALYSIS\n" << std::endl;
    std::cout << boost::format("Total Entries: %d, %d entries were not in the bag file\n")
        % totalEntries % (csvVec.size() - 1 - totalEntries) << std::endl;
    std::cout << "Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMean << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMean << std::endl;

    std::cout << "Not Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMeanOutliers << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMeanOutliers << std::endl;
}

unsigned closest(const std::vector<std::vector<double>> &vec, double val) {
    if (val <= vec.at(0).at(timeIndex))
        return findValid(vec, 0);
    if (val >= vec.at(vec.size() - 1).at(timeIndex))
        return findValid(vec, vec.size() - 1);

    unsigned i = 0, j = vec.size() - 1, mid = 0;
    while (i < j) {
        mid = (i + j) / 2;
        if (vec.at(mid).at(timeIndex) == val)
            return findValid(vec, mid);
        if (vec.at(mid).at(timeIndex) > val) {
            j = mid;
        }
        else {
            i = mid + 1;
        }
    }
    return findValid(vec, mid);
}

unsigned findValid(const std::vector<std::vector<double>> &vec, unsigned index) {
    if(!isnan(vec.at(index).at(yawIndex))) {
        return index;
    }
    if(index >= msgSearchRadius && index < vec.size() - msgSearchRadius) {
        for(unsigned i{index - msgSearchRadius}; i < index + msgSearchRadius; i++) {
            if(i != index && (!isnan(vec.at(i).at(yawIndex))) && (vec.at(index).at(timeIndex) == vec.at(i).at(timeIndex))) {
                return i;
            }
        }
    }
    return index;
}