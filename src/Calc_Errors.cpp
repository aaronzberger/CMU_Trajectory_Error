#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

unsigned findInBag(const std::vector<double> &csvMsg, const std::vector<std::vector<double>> &bag, unsigned lastBagIndex);

const unsigned xIndex = 0;
const unsigned yIndex = 1;
const unsigned yawIndex = 2;

const unsigned wayptIndexIndex = 0;
const unsigned positionErrorIndex = 1;
const unsigned orientationErrorIndex = 2;
const unsigned bagMsgIndex = 3;

const double zScoreSignificanceLevel = 3;

const unsigned bagSearchLength = 70;
const unsigned minMoveValue = 5;

const unsigned outlierAtPosition = 0;
const unsigned outlierAtOrientation = 1;
const unsigned outlierAtBoth = 2;

const unsigned indexInErrorVecIndex = 0;
const unsigned outlierIdentifierIndex = 1;

const double rosTimeConversionFactor = pow(10, -9);

const unsigned csvStartIdx = 0;
const unsigned csvEndIdx = 6940;

const unsigned bagStartIdx = 1580;
const unsigned bagEndIdx = 91210;

int main(int argc, char **argv) {

    std::ofstream csvPrint("csvPrint.csv");
    std::ofstream bagPrint("bagPrint.csv");

    //Ensure 2 arguments were passed in, or throw an error
    if(argc != 3) {
        perror("Error: Expected two arguments: CSV file path, BAG folder path\n");
        return 1;
    }

    //Open the CSV file from the passed-in path
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        perror("Error: Could not open CSV File\n");
        return 1;
    }

    //Declare holding variables for the values of each column
    std::string x, y, yaw;

    //Parse out the first line, which are the titles of the columns
    std::getline(file, x, '\n');

    std::vector<std::vector<double>> csvVec;
    unsigned currentIdx {0};

    //Copy the csv entries to the vector csvVec while calculating the time for each entry.
    while (file.good()) {
        std::getline(file, x, ',');
        std::getline(file, y, ',');
        std::getline(file, yaw, '\n');

        if(file.eof()) break;

        if(currentIdx >= csvStartIdx && currentIdx <= csvEndIdx) {
            std::vector<double> instance;
            instance.push_back(std::stod(x));
            instance.push_back(std::stod(y));
            instance.push_back(std::stod(yaw));

            csvVec.push_back(instance);

            csvPrint << x << "," << y << "," << yaw << "," << currentIdx << "\n";
        }
        currentIdx++;
    }
    file.close();
    csvPrint.close();

    //Open the bag files from the passed-in path
    const boost::filesystem::path bagDirPath(argv[2]);

    std::vector<boost::filesystem::directory_entry> bagFiles;

    if(!boost::filesystem::exists(bagDirPath)) {
        perror("Unable to open bag directory");
        return 1;
    }
    if(boost::filesystem::is_directory(bagDirPath)) {

        for(const auto file : boost::filesystem::directory_iterator(bagDirPath)) {
            bagFiles.push_back(file);
        }

        std::sort(bagFiles.begin(), bagFiles.end());
    } else {
        perror("Second argument must be a path to the bag directory");
    }

    std::vector<std::vector<double>> bagVec;

    currentIdx = 0;

    for(auto &bagFile : bagFiles) {
        rosbag::Bag bag;
        std::cout << "\nReading " << bagFile.path().filename().string();
        bag.open(bagFile.path().string());

        // Preparation for printing a loading symbol
        std::vector<char> loading {'/','-','\\','|','/','-','\\','|'};

        unsigned currentLoadingIdx {0};
        std::cout << " " << loading.at(currentLoadingIdx);

        rosbag::View viewer(bag);
        // Copy the bag entries to the vector bagVec while converting the quaternion to a yaw measurement
        for (const rosbag::MessageInstance msg : viewer) {
            if(msg.getDataType() == "nav_msgs/Odometry") {
                if(currentIdx >= bagStartIdx && currentIdx <= bagEndIdx) {
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

                        bagPrint << thisMsg->pose.pose.position.x << "," << thisMsg->pose.pose.position.y << "," << yaw << "," << currentIdx << "\n";

                        // Update the loading symbol periodically
                        if((currentIdx % 200) == 0) {
                            if(currentLoadingIdx + 1 > loading.size() - 1) currentLoadingIdx = 0;
                            else currentLoadingIdx++;
                            std::cout <<  "\b";
                            std::cout.flush();
                            std::cout << loading.at(currentLoadingIdx);
                            std::cout.flush();
                        }
                    }
                    else {
                        std::cout << "Error: Could not retrieve this bag message. " << msg.getDataType() << std::endl;
                    }
                }
                currentIdx++;
            }
        }
        bag.close();

        // Clear the text from the loading symbol before moving on
        std::cout << "\b" << " ";
        std::cout.flush();
    }
    bagPrint.close();

    std::cout << std::endl;

    double positionErrorTotal{0};
    double orientationErrorTotal{0};

    std::vector<std::vector<double>> errorVec;

    unsigned lastBagIndex {0};

    //Loop through csvVec and find the corresponding bag entries. Calculate the error and copy it to errorVec
    for(unsigned i{0}; i < csvVec.size(); i++) {
        if(lastBagIndex + 1 >= bagVec.size()) break;
        unsigned bagMsgIndex{findInBag(csvVec.at(i), bagVec, lastBagIndex + 1)}; 
        lastBagIndex = bagMsgIndex;
        //Calculate position error using Euclidian distance
        double positionError{std::sqrt(pow((csvVec.at(i).at(xIndex) - bagVec.at(bagMsgIndex).at(xIndex)), 2) + 
                                       pow((csvVec.at(i).at(yIndex) - bagVec.at(bagMsgIndex).at(yIndex)), 2))};
        double orientationError{std::abs(csvVec.at(i).at(yawIndex) - bagVec.at(bagMsgIndex).at(yawIndex))};

        //Correct orientation error if large error is due to radian looping
        if(orientationError > M_PI && orientationError < M_PI * 2) orientationError = (M_PI * 2) - orientationError;

        std::vector<double> instance;

        instance.push_back(i);
        instance.push_back(positionError);
        instance.push_back(orientationError);
        instance.push_back(bagMsgIndex);

        errorVec.push_back(instance);


        positionErrorTotal += positionError;
        orientationErrorTotal += orientationError;
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

    //Print data

    //Print the individual error entries
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "INDIVIDUAL ENTRIES\n" << std::endl;
    for (auto vec : errorVec) {
        std::cout << boost::format("Waypt Index: [%4p], Bag Index: [%5p], Position Error: [%07.5f], Orientation Error: [%07.5f]") 
            % vec.at(wayptIndexIndex) % vec.at(3) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex)<< std::endl;
    }

    //Print the outliers
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "OUTLIERS\n" << std::endl;

    for(auto vec: outlierVec) {
        std::cout << boost::format("Waypt Index: [%4p], ") % errorVec.at(vec.at(0)).at(wayptIndexIndex);
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

    //Print the entry count and the means for position and orientation
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "ANALYSIS\n" << std::endl;
    std::cout << boost::format("Total Entries: %d, failed to find %d entries in the bag file\n")
        % errorVec.size() % (csvVec.size() - errorVec.size()) << std::endl;
    std::cout << boost::format("Found %d outliers\n") % outlierVec.size() << std::endl;
    std::cout << "Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMean << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMean << std::endl;

    std::cout << "Not Counting Outliers:" << std::endl;
    std::cout << boost::format("Position Error Mean: [%.5f]") % positionErrorMeanOutliers << std::endl;
    std::cout << boost::format("Orientation Error Mean: [%.5f]\n") % orientationErrorMeanOutliers << std::endl;


    //Write a new csv file for the errors
    std::string bagFolderName {bagDirPath.filename().string()};
    std::ofstream errorCSVFile(bagFolderName.append("_error.csv"));

    //Write the analysis (means and entry count)
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

    //Write the outliers
    errorCSVFile << "OUTLIERS" << "\n";
    errorCSVFile << "Waypt Index, Bag Index, Position Error, Orientation Error" << "\n";

    for(auto vec: outlierVec) {
        errorCSVFile << boost::format("%4p, %5p,") % errorVec.at(vec.at(0)).at(wayptIndexIndex) % errorVec.at(vec.at(0)).at(bagMsgIndex);
        if(vec.at(outlierIdentifierIndex) == outlierAtPosition || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format("%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(positionErrorIndex);
        }
        if(vec.at(outlierIdentifierIndex) == outlierAtOrientation || vec.at(outlierIdentifierIndex) == outlierAtBoth) {
            errorCSVFile << boost::format(",%-.5f") % errorVec.at(vec.at(indexInErrorVecIndex)).at(orientationErrorIndex);
        }
        errorCSVFile << "\n";
    }

    //Write the column headers
    errorCSVFile << "\n\nINDIVIDUAL ENTRIES\n";
    errorCSVFile << "Waypt Index, Bag Index, Position Error, Orientation Error" << "\n";

    //Write the individual entries
    for (auto vec : errorVec) {
        errorCSVFile << boost::format("%4p,%5p,%-.5f,%-.5f") 
            % vec.at(wayptIndexIndex) % vec.at(bagMsgIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }
    errorCSVFile << "\n\n";


    errorCSVFile.close();

    //Write a new csv file for the gnu plot
    std::ofstream errorGraphCSVFile(bagFolderName.append("_error_graph_data.csv"));

    //Write the individual entries, adjusting time so it is viewable on a graph
    for (auto vec : errorVec) {
        errorGraphCSVFile << boost::format("%4p,%.5f,%.5f") 
            % vec.at(wayptIndexIndex) % vec.at(positionErrorIndex) % vec.at(orientationErrorIndex) << std::endl;
    }

    return 0;
}

/**
 * @brief Gets the index of the message in the bag that has the given time stamp
 * 
 * @param vec the bag vector in which to look for the time stamp
 * @param val the time stamp to look for
 * @return the index of the message with that time stamp
 * @throw std::out_of_range if bagVec was not created properly or columns are missing
 */
unsigned findInBag(const std::vector<double> &csvMsg, const std::vector<std::vector<double>> &bag, unsigned lastBagIndex) {
    unsigned closestIndex {lastBagIndex};
    double closestDistance {std::numeric_limits<double>::max()};

    unsigned endSearchIndex {lastBagIndex + bagSearchLength};
    if(endSearchIndex >= bag.size() - 1) endSearchIndex = bag.size() - 1;

    unsigned beginSearchIndex {lastBagIndex};
    if(!(beginSearchIndex < minMoveValue)) beginSearchIndex += minMoveValue;

    for(unsigned i{beginSearchIndex}; i <= endSearchIndex; i++) {
        double distance {std::sqrt(pow((csvMsg.at(xIndex) - bag.at(i).at(xIndex)), 2) + 
                                   pow((csvMsg.at(yIndex) - bag.at(i).at(yIndex)), 2))};
        if(distance < closestDistance && !std::isnan(bag.at(i).at(yawIndex))) {
            closestDistance = distance;
            closestIndex = i;
        }
    }
    return closestIndex;
}