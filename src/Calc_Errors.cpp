#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/format.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

unsigned findClosestWaypt(const std::vector<double> &bagMsg, const std::vector<std::vector<double>> &csvVec, unsigned predictedLocation);
std::string getFileName(const std::string &full_path);

// Indexes for input file
const unsigned xIndex = 0;
const unsigned yIndex = 1;
const unsigned yawIndex = 2;

// Indexes for error vector
const unsigned bagMsgIndexIndex = 0;
const unsigned positionErrorIndex = 1;
const unsigned orientationErrorIndex = 2;
const unsigned wayptMsgIndexIndex = 3;

const double zScoreSignificanceLevel = 3;

const unsigned wayptSearchRadius = 70;

const unsigned outlierAtPosition = 0;
const unsigned outlierAtOrientation = 1;
const unsigned outlierAtBoth = 2;

const unsigned indexInErrorVecIndex = 0;
const unsigned outlierIdentifierIndex = 1;

const unsigned bagMsgsAreCloseThreshold = 3;

bool calculateStoppingPoints;

const std::vector<unsigned> stoppingBagIndexes {2480, 3615, 4905, 6780, 9080, 10825, 12040, 13735, 15405, 16865, 18060, 19295, 21410, 23935, 25920, 27435, 28860, 30095, 31265, 32830};

int main(int argc, char **argv) {

    // Ensure 2 arguments were passed in, or throw an error
    if(argc != 3 && argc != 4) {
        perror("Error: Expected two or three arguments: CSV file path, BAG file path, and an optional stopping points file path\n");
        return 1;
    }

    if(argc == 4) calculateStoppingPoints = true;

    // Open the CSV file from the passed-in path
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        perror("Error: Could not open CSV File\n");
        return 1;
    }

    std::string x, y, yaw;

    // Parse out the first line, which are the titles of the columns
    std::getline(file, x, '\n');

    std::vector<std::vector<double>> csvVec;

    // Copy the csv entries to the vector csvVec
    while (file.good()) {
        std::getline(file, x, ',');
        std::getline(file, y, ',');
        std::getline(file, yaw, '\n');

        if(file.eof()) break;

        std::vector<double> instance;
        instance.push_back(std::stod(x));
        instance.push_back(std::stod(y));
        instance.push_back(std::stod(yaw));

        csvVec.push_back(instance);
    }
    file.close();


    std::vector<std::vector<double>> bagVec;

    rosbag::Bag bag;
    bag.open(argv[2]);

    rosbag::View viewer(bag);
    // Copy the bag entries to the vector bagVec while converting the quaternion to a yaw measurement
    for (const rosbag::MessageInstance msg : viewer) {
        if(msg.getDataType() == "nav_msgs/Odometry") {
            nav_msgs::Odometry::ConstPtr thisMsg = msg.instantiate<nav_msgs::Odometry>();
            if (thisMsg != nullptr) {
                tf::Quaternion quaternion;
                quaternion.setW(thisMsg->pose.pose.orientation.w);
                quaternion.setX(thisMsg->pose.pose.orientation.x);
                quaternion.setY(thisMsg->pose.pose.orientation.y);
                quaternion.setZ(thisMsg->pose.pose.orientation.z);

                double yaw {tf::getYaw(quaternion)};
                
                std::vector<double> instance;

                instance.push_back(thisMsg->pose.pose.position.x);
                instance.push_back(thisMsg->pose.pose.position.y);
                instance.push_back(yaw);

                bagVec.push_back(instance);
            }
            else {
                std::cout << "Error: Could not retrieve this bag message of type " << msg.getDataType() << std::endl;
            }
        }
    }
    bag.close();


    // Filter out entries where yaw is nan or the measurements are incorrect
    // Procedure: find the mode x value from the last 10 entries; eliminate new entries that aren't close to that mode
    std::vector<std::vector<double>> filteredBagVec;

    double xMode;

    double val1 {std::nan("")};
    double val2 {std::nan("")};
    unsigned val1Counter {0};
    unsigned val2Counter {0};

    // Find the x mode for the first 10 values
    for(int i{0}; i <= 10; i++) {
        if(std::isnan(val1)) {
            val1 = bagVec.at(i).at(xIndex);
            continue;
        }
        if(std::isnan(val2)) {
            val2 = bagVec.at(i).at(xIndex);
            continue;
        }
        if(std::abs(bagVec.at(i).at(xIndex) - val1) < bagMsgsAreCloseThreshold) val1Counter++;
        if(std::abs(bagVec.at(i).at(xIndex) - val2) < bagMsgsAreCloseThreshold) val2Counter++;
    }

    xMode = val1Counter > val2Counter ? val1 : val2;

    // Filter out all the invalid entries
    for(int i{0}; i < bagVec.size(); i++) {
        if(!std::isnan(bagVec.at(i).at(yawIndex)) &&
           std::abs(bagVec.at(i).at(xIndex) - xMode) < bagMsgsAreCloseThreshold) {
            filteredBagVec.push_back(bagVec.at(i));
            xMode = bagVec.at(i).at(xIndex);
        }
    }


    // Find the error for each waypoint
    // Procedure: Find the bag message with the least error for each waypoint, eliminating flat sections where the robot was stopped

    // Use a hash map that maps the waypoint index to a vector containing the errors
    std::unordered_map<unsigned, std::vector<double>> closestDistances;

    // Loop through bagVec and find the corresponding waypoints
    for(unsigned i{0}; i < filteredBagVec.size(); i++) {
        unsigned predictedLocation {i / (filteredBagVec.size() / csvVec.size())};

        unsigned csvMsgIndex{findClosestWaypt(filteredBagVec.at(i), csvVec, predictedLocation)};

        // Calculate position error using Euclidian distance
        double positionError{std::sqrt(pow((filteredBagVec.at(i).at(xIndex) - csvVec.at(csvMsgIndex).at(xIndex)), 2) + 
                                       pow((filteredBagVec.at(i).at(yIndex) - csvVec.at(csvMsgIndex).at(yIndex)), 2))};
        double orientationError{std::abs(filteredBagVec.at(i).at(yawIndex) - csvVec.at(csvMsgIndex).at(yawIndex))};

        // Correct orientation error if large error is due to radian looping
        if(orientationError > M_PI && orientationError < M_PI * 2) orientationError = (M_PI * 2) - orientationError;

        std::vector<double> instance;

        instance.push_back(i);
        instance.push_back(positionError);
        instance.push_back(orientationError);
        instance.push_back(csvMsgIndex);

        // Decide whether to use this entry (if it has the least error for the found waypoint)
        if(closestDistances.count(csvMsgIndex) == 0) {
            closestDistances.insert({csvMsgIndex, instance});
        } else {
            std::unordered_map<unsigned, std::vector<double>>::const_iterator element = closestDistances.find(csvMsgIndex);
            if(positionError < element->second.at(positionErrorIndex)) {
                closestDistances.erase(csvMsgIndex);
                closestDistances.insert({csvMsgIndex, instance});
            }
        }
    }


    // Copy the contents of the hash map to a vector containing the errors, sorted by waypoint message index
    std::vector<std::vector<double>> errorVec;

    double positionErrorTotal{0};
    double orientationErrorTotal{0};

    for(int i{0}; i < csvVec.size(); i++) {
        if(closestDistances.count(i) == 1) {
            std::unordered_map<unsigned, std::vector<double>>::const_iterator element = closestDistances.find(i);
            errorVec.push_back(element->second);
            positionErrorTotal += element->second.at(positionErrorIndex);
            orientationErrorTotal += element->second.at(orientationErrorIndex);
        }
    }


    // Do pre-calculations for finding outliers
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

    //Loop through errorVec and find outlier entries. Store those entries in a vector (outlierVec)
    for(int i {0}; i < errorVec.size(); i++) {
        double positionErrorZScore {(errorVec.at(i).at(positionErrorIndex) - positionErrorMean) / positionErrorSD};
        double orientationErrorZScore {(errorVec.at(i).at(orientationErrorIndex) - orientationErrorMean) / orientationErrorSD};

        if(std::abs(positionErrorZScore) > zScoreSignificanceLevel || std::abs(orientationErrorZScore) > zScoreSignificanceLevel) {
            std::vector<int> instance;
            instance.push_back(i);
            if(std::abs(positionErrorZScore) > zScoreSignificanceLevel && std::abs(orientationErrorZScore) > zScoreSignificanceLevel) {
                instance.push_back(outlierAtBoth);
            } else if(std::abs(positionErrorZScore) > zScoreSignificanceLevel) {
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

    double rmse;

    // Calculate statistics for the stopping points, if desired
    if(calculateStoppingPoints) {
        // Open the CSV file from the passed-in path
        std::ifstream stoppingPtsFile(argv[3]);
        if (!stoppingPtsFile.is_open()) {
            perror("Error: Could not open Stopping Points File\n");
            return 1;
        }

        std::string x, y;

        // Parse out the first line, which are the titles of the columns
        std::getline(stoppingPtsFile, x, '\n');

        std::vector<std::vector<double>> stoppingPointsVec;

        // Copy the entries to the vector stoppingPointsVec
        while (stoppingPtsFile.good()) {
            std::getline(stoppingPtsFile, x, ',');
            std::getline(stoppingPtsFile, y, '\n');

            if(stoppingPtsFile.eof()) break;

            std::vector<double> instance;
            instance.push_back(std::stod(x));
            instance.push_back(std::stod(y));

            stoppingPointsVec.push_back(instance);
        }
        stoppingPtsFile.close();

        std::vector<double> residuals;

        for(int i{0}; i < stoppingBagIndexes.size(); i++) {
            residuals.push_back(pow((filteredBagVec.at(stoppingBagIndexes.at(i)).at(xIndex) - stoppingPointsVec.at(i).at(xIndex)), 2) + 
                                pow((filteredBagVec.at(stoppingBagIndexes.at(i)).at(yIndex) - stoppingPointsVec.at(i).at(yIndex)), 2));    
        }
        rmse = std::sqrt(std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size());

        std::ofstream stoppingPtsOutput(getFileName(argv[3]).append("_results.csv"));

        unsigned pointNumber {1};
        stoppingPtsOutput << "Stopping Point" << "," << "Distance Error" << "\n";
        for(auto resid : residuals) {
            stoppingPtsOutput << pointNumber++ << "," << std::sqrt(resid) << "\n";
        }
        stoppingPtsOutput << "\n";

        stoppingPtsOutput << "RMSE," << rmse << std::endl;
        stoppingPtsOutput.close();
    }

    // Print all the results
    // Print the individual error entries
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "INDIVIDUAL ENTRIES\n" << std::endl;
    for (auto vec : errorVec) {
        std::cout << boost::format("Bag Index: [%5p], Waypt Index: [%4p], Position Error: [%07.5f], Orientation Error: [%07.5f]") 
            % vec.at(0) % vec.at(3) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }

    // Print the outliers
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "OUTLIERS\n" << std::endl;

    for(auto vec: outlierVec) {
        std::cout << boost::format("Bag Index: [%4p], ") % errorVec.at(vec.at(0)).at(bagMsgIndexIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format("Position Error: [%07.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation) {
            std::cout << boost::format("%27sOrientation Error: [%07.5f]") % "" % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            std::cout << boost::format(", Orientation Error: [%07.5f]") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        std::cout << std::endl;
    }

    // Print the analysis (entry count and the means for position and orientation)
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "ANALYSIS\n" << std::endl;
    std::cout << boost::format("Total Entries: %d\n")
        % errorVec.size() << std::endl;
    std::cout << "Filtered out " << bagVec.size() - filteredBagVec.size() << " invalid entries from the original bag file (out of " << bagVec.size() << " total entries)\n\n";
    std::cout << boost::format("Found %d outliers\n") % outlierVec.size() << std::endl;
    std::cout << "Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMean << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMean << std::endl;

    std::cout << "Not Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMeanOutliers << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMeanOutliers << std::endl;


    // Write a new csv file for the errors
    std::ofstream errorCSVFile(getFileName(argv[2]).append("_error.csv"));

    // Write the analysis (means and entry count)
    errorCSVFile << "ANALYSIS" << "\n";

    errorCSVFile << "Total Entries," << errorVec.size() << "\n";
    errorCSVFile << "Entries not Found," << (csvVec.size() - errorVec.size()) << "\n";
    errorCSVFile << "Outliers Found, " << outlierVec.size() << "\n\n";
    errorCSVFile << "Counting Outliers" << "\n";
    errorCSVFile << boost::format("Position Error Mean,%-.7f") % positionErrorMean << "\n";
    errorCSVFile << boost::format("Orientation Error Mean,%-.7f") % orientationErrorMean << "\n\n";

    errorCSVFile << "Not Counting Outliers" << "\n";
    errorCSVFile << boost::format("Position Error Mean,%-.7f") % positionErrorMeanOutliers << "\n";
    errorCSVFile << boost::format("Orientation Error Mean,%-.7f") % orientationErrorMeanOutliers << "\n\n\n";

    // Write the outliers
    errorCSVFile << "OUTLIERS" << "\n";
    errorCSVFile << "Bag Index, Waypt Index, Position Error, Orientation Error" << "\n";

    for(auto vec: outlierVec) {
        errorCSVFile << boost::format("%4p, %5p,") % errorVec.at(vec.at(0)).at(bagMsgIndexIndex) % errorVec.at(vec.at(0)).at(wayptMsgIndexIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format("%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format(",%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        errorCSVFile << "\n";
    }

    // Write column headers
    errorCSVFile << "\n\nINDIVIDUAL ENTRIES\n";
    errorCSVFile << "Bag Index, Waypt Index, Position Error, Orientation Error" << "\n";

    // Write the individual entries
    for (auto vec : errorVec) {
        errorCSVFile << boost::format("%4p,%5p,%-.5f,%-.5f") 
            % vec.at(bagMsgIndexIndex) % vec.at(wayptMsgIndexIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }
    errorCSVFile.close();


    // Write a new csv file for graphing
    std::ofstream errorGraphCSVFile(getFileName(argv[2]).append("_error_graph_data.csv"));

    // Write the individual entries, using the waypoint index as the time stamp
    for (auto vec : errorVec) {
        errorGraphCSVFile << boost::format("%4p,%.5f,%.5f") 
            % vec.at(wayptMsgIndexIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }

    return 0;
}

/**
 * @brief Gets the index of the waypoint that is closest to the given bag entry, within a radius around the predicted location
 * 
 * @param bagMsg the message from the bag file
 * @param csvVec a vector containing all the waypoints
 * @param predictedLocation a predicted location of the waypoint
 * @return the index of the waypoint closest to the bag message
 * @throw std::out_of_range if csvVec was not created properly or columns are missing
 */
unsigned findClosestWaypt(const std::vector<double> &bagMsg, const std::vector<std::vector<double>> &csvVec, unsigned predictedLocation) {
    unsigned closestIndex {0};
    double closestDistance {std::numeric_limits<double>::max()};

    unsigned startLocation {0};
    if(static_cast<int>(predictedLocation) - wayptSearchRadius > 0) startLocation = predictedLocation - wayptSearchRadius;

    unsigned endLocation{csvVec.size() - 1};
    if(static_cast<int>(predictedLocation) + wayptSearchRadius < csvVec.size() - 1) endLocation = predictedLocation + wayptSearchRadius;

    for(unsigned i{startLocation}; i <= endLocation; i++) {
        double distance {std::sqrt(pow((bagMsg.at(xIndex) - csvVec.at(i).at(xIndex)), 2) + 
                                   pow((bagMsg.at(yIndex) - csvVec.at(i).at(yIndex)), 2))};
        if(distance < closestDistance) {
            closestIndex = i;
            closestDistance = distance;
        }
    }
    return closestIndex;
}

/**
 * @brief Finds the directory name from a path
 * 
 * @param path the path on which to find the directory name
 * @return a string containing the directory name
 */
std::string getFileName(const std::string &full_path) {
    std::string file_name {full_path.substr(full_path.find_last_of("/") + 1)};
    file_name = file_name.substr(0, file_name.rfind('.'));
    return file_name;
}