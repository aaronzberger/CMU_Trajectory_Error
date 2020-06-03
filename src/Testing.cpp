#include <vector>
#include <iostream>

int closest(const std::vector<std::vector<double>> &vec, double val);
int getClosest(const std::vector<std::vector<double>> &vec, int val1, int val2, double val);

int main(int argc, char **argv)
{
    std::vector<std::vector<double>> hey{
        {5.5, 6.7, 9.3},
        {6.4, 7.5, 8.5},
        {8.3, 6.7, 9.3},
        {9.1, 6.7, 9.3},
        {10.5, 6.7, 9.3},
        {12.6, 6.7, 9.3},
        {15.7, 6.7, 9.3},
        {15.9, 6.7, 9.3}};
    double target{15.7};
    int answer{closest(hey, target)};
    std::cout.precision(5);
    std::cout << answer << std::endl;
}

int closest(const std::vector<std::vector<double>> &vec, double val)
{
    if (val <= vec.at(0).at(0))
        return 0;
    if (val >= vec.at(vec.size() - 1).at(0))
        return vec.size() - 1;

    int i = 0, j = vec.size() - 1, mid = 0;
    while (i < j)
    {
        mid = (i + j) / 2;
        if (vec.at(mid).at(0) == val)
            return mid;
        if (vec.at(mid).at(0) > val)
        {
            // if (mid > 0 && val > vec.at(mid - 1).at(0))
            // {
            //     return getClosest(vec, mid - 1, mid, val);
            // }
            j = mid;
        }
        else
        {
            // if (mid < vec.size() - 1 && val < vec.at(mid + 1).at(0))
            // {
            //     return getClosest(vec, mid, mid + 1, val);
            // }
            i = mid + 1;
        }
    }
    return vec.at(mid).at(0);
}

int getClosest(const std::vector<std::vector<double>> &vec, int val1, int val2, double val)
{
    if (abs(val - vec.at(val2).at(0)) >= abs(vec.at(val2).at(0) - val))
        return val2;
    else
        return val1;
}

// //Print bag file contents for testing
// for(int i {0}; i < 2000; i++) {
//     std::cout << "Time Stamp: [" << bagVec.at(i).at(0) << "], Yaw Calculated: [" << bagVec.at(i).at(3) << "].\n";
// }


// //Old printing without formatting
    // std::cout.precision(20);
    // for (auto vec : errorVec) {
    //     std::cout << "Time: [" << vec.at(0) << "], Pos Error: [" << vec.at(1) << "], Orient Error: [" << vec.at(2) << "]" << std::endl;
    // }
    
    // std::cout << "Position Error Mean: [" << positionErrorTotal / totalEntries << "]" << std::endl;
    // std::cout << "Position Error Mean: [" << orientationErrorTotal / totalEntries << "]" << std::endl;
    // std::cout << "Total Entries: " << totalEntries << ", " << 
    //     csvVec.size() - 1 - totalEntries << " entries were not in the bag file" <<std::endl;
