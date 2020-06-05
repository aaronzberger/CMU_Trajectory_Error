#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/format.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

unsigned findInBag(const std::vector<std::vector<double>> &vec, double val);
unsigned findValid(const std::vector<std::vector<double>> &vec, unsigned index);
std::string getBagName(const std::string &full_path);

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

const double rosTimeConversionFactor = pow(10, -9);

int main(int argc, char **argv) {

    //Ensure 2 arguments were passed in, or throw an error
    if(argc != 3) {
        perror("Error: Expected two arguments: CSV file path, BAG file path\n");
        return 1;
    }

    //Open the CSV file from the passed-in path
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        std::cout << "Could not open CSV File" << std::endl;
        return 1;
    }

    //Declare holding variables for the values of each column
    std::string secs, nsecs, x, y, yaw;

    //Parse out the first line, which are the titles of the columns
    std::getline(file, secs, '\n');

    std::vector<std::vector<double>> csvVec;

    //Copy the csv entries to the vector csvVec while calculating the time for each entry.
    while (file.good()) {
        std::getline(file, secs, ',');
        std::getline(file, nsecs, ',');
        std::getline(file, x, ',');
        std::getline(file, y, ',');
        std::getline(file, yaw, '\n');

        if(file.eof()) break;

        std::vector<double> instance;

        instance.push_back(std::stod(secs) + (std::stod(nsecs) * rosTimeConversionFactor));
        instance.push_back(std::stod(x));
        instance.push_back(std::stod(y));
        instance.push_back(std::stod(yaw));

        csvVec.push_back(instance);
    }
    file.close();

    //Open the bag file from the passed-in path
    rosbag::Bag bag;
    bag.open(argv[2]);

    rosbag::View viewer(bag);

    std::vector<std::vector<double>> bagVec;

    //Copy the bag entries to the vector bagVec while converting the quaternion to a yaw measurement
    for (const rosbag::MessageInstance msg : viewer) {
        nav_msgs::Odometry::ConstPtr thisMsg = msg.instantiate<nav_msgs::Odometry>();
        if (thisMsg != nullptr) {
            tf::Quaternion quaternion;
            quaternion.setW(thisMsg->pose.pose.orientation.w);
            quaternion.setX(thisMsg->pose.pose.orientation.x);
            quaternion.setY(thisMsg->pose.pose.orientation.y);
            quaternion.setZ(thisMsg->pose.pose.orientation.z);

            double yaw {tf::getYaw(quaternion)};
            std::vector<double> instance;

            instance.push_back(thisMsg->header.stamp.sec + (thisMsg->header.stamp.nsec * rosTimeConversionFactor));
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

    double positionErrorTotal{0};
    double orientationErrorTotal{0};

    std::vector<std::vector<double>> errorVec;

    //Loop through csvVec and find the corresponding bag entries. Calculate the error and copy it to errorVec
    for (auto csvMsg : csvVec) {
        unsigned bagMsgIndex{findInBag(bagVec, csvMsg.at(timeIndex))};       
        if (bagVec.at(bagMsgIndex).at(timeIndex) == csvMsg.at(timeIndex)) {
            //Calculate position error using Euclidian distance
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
        }
    }

    double positionErrorMean {positionErrorTotal / errorVec.size()};
    double orientationErrorMean {orientationErrorTotal / errorVec.size()};

    double positionErrorSD {0};
    double orientationErrorSD {0};

    //Loop through errorVec and calculate the standard deviation for position error and orientation error
    for(int i {0}; i < errorVec.size(); i++) {
        positionErrorSD += pow(errorVec.at(i).at(positionErrorIndex), 2);
        orientationErrorSD += pow(errorVec.at(i).at(orientationErrorIndex), 2);
    }
    positionErrorSD /= errorVec.size();
    orientationErrorSD /= errorVec.size();

    positionErrorSD = sqrt(positionErrorSD);
    orientationErrorSD = sqrt(orientationErrorSD);


    std::vector<std::vector<int>> outlierVec;

    double positionErrorMeanOutliers {0};
    int positionErrorMeanCount {0};
    double orientationErrorMeanOutliers {0};
    int orientationErrorMeanCount {0};

    //Loop through errorVec and find outlier entries. Store those entries in outlierVec
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
            outlierVec.push_back(instance);
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

    //Print the individual error entries
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "INDIVIDUAL ENTRIES\n" << std::endl;
    for (auto vec : errorVec) {
        std::cout << boost::format("Time: [%.3f], Position Error: [%.5f], Orientation Error: [%.5f]") 
            % vec.at(timeIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }

    //Print the outliers
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "OUTLIERS\n" << std::endl;

    for(auto vec: outlierVec) {
        std::cout << boost::format("Stamp: [%.3f], ") % errorVec.at(vec.at(0)).at(timeIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format("Position Error: [%.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format("Orientation Error: [%.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        std::cout << std::endl;
    }

    //Print the entry count and the means for position and orientation
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "ANALYSIS\n" << std::endl;
    std::cout << boost::format("Total Entries: %d, %d entries were not in the bag file\n")
        % errorVec.size() % (csvVec.size() - 1 - errorVec.size()) << std::endl;
    std::cout << "Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMean << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMean << std::endl;

    std::cout << "Not Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMeanOutliers << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMeanOutliers << std::endl;


    //Write a new csv file for the errors
    std::ofstream errorCSVFile(getBagName(argv[2]).append("_error.csv"));

    //Write the column headers
    errorCSVFile << "Time Stamp, Position Error, Orientation Error" << "\n";

    //Write the individual entries
    for (auto vec : errorVec) {
        errorCSVFile << boost::format("%-.5f,%-.5f,%-.5f") 
            % vec.at(timeIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }
    errorCSVFile << "\n\n";

    //Write the outliers
    errorCSVFile << "OUTLIERS" << "\n";

    for(auto vec: outlierVec) {
        errorCSVFile << boost::format("%-.3f,") % errorVec.at(vec.at(0)).at(timeIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format("%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format(",%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        errorCSVFile << "\n";
    }

    //Write the analysis (means and entry count)
    errorCSVFile << "\nANALYSIS" << "\n\n";

    errorCSVFile << "Total Entries," << errorVec.size() << "\n";
    errorCSVFile << "Entries not Found," << (csvVec.size() - 1 - errorVec.size()) << "\n\n";
    errorCSVFile << "Counting Outliers" << "\n";
    errorCSVFile << boost::format("Position Error Mean,%-.7f") % positionErrorMean << "\n";
    errorCSVFile << boost::format("Orientation Error Mean,%-.7f") % orientationErrorMean << "\n\n";

    errorCSVFile << "Not Counting Outliers" << "\n";
    errorCSVFile << boost::format("Position Error Mean,%-.7f") % positionErrorMeanOutliers << "\n";
    errorCSVFile << boost::format("Orientation Error Mean,%-.7f") % orientationErrorMeanOutliers << "\n";
    errorCSVFile.close();

    //Write a new csv file for the gnu plot
    std::ofstream errorGraphCSVFile(getBagName(argv[2]).append("_error_graph_data.csv"));

    //Write the individual entries, adjusting time so it is viewable on a graph
    double startingTime {errorVec.at(0).at(timeIndex)};
    for (auto vec : errorVec) {
        errorGraphCSVFile << boost::format("%.5f,%.5f,%.5f") 
            % (vec.at(timeIndex) - startingTime) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }

    return 0;
}

/**
 * @brief Finds the name of a file from the absolute path
 * 
 * @param full_path the full path of the file, including the name and extension
 * @return the name of the file
 * @throw std::out_of_range if no "/"" is found in the full path,
 *                          if no "." is found in the file name
 */
std::string getBagName(const std::string &full_path) {
    std::string file_name {full_path.substr(full_path.find_last_of("/") + 1)};
    file_name = file_name.substr(0, file_name.rfind('.'));
    return file_name;
}

/**
 * @brief Gets the index of the message in the bag that has the given time stamp
 * 
 * @param vec the bag vector in which to look for the time stamp
 * @param val the time stamp to look for
 * @return the index of the message with that time stamp
 * @throw std::out_of_range if bagVec was not created properly or columns are missing
 */
unsigned findInBag(const std::vector<std::vector<double>> &vec, double val) {
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

/**
 * @brief Helper function to findInBag that corrects for invalid entries in the bag
 * 
 * @param vec the bag vector the index refers to
 * @param index the index in the vector of the already-chosen message
 * @return the index of the message with the correct time stamp
 * @throw std::out_of_range if bagVec was not created properly or columns are missing
 */
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