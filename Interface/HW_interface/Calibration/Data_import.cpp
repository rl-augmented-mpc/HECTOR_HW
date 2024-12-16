#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "orientation_tools.h"

using namespace std;

size_t count_lines_angle(const char *filename)
{
    ifstream myfile("../build/angle.txt");
    string line;
    size_t count = 0;
    while (getline(myfile,line))
    {
        ++count;
    }
    return count;
}

void angle_offset(){
     //Angle Offset Calculation
    const char filename[] = "../build/angle.txt";
    size_t i, count = count_lines_angle(filename);
    ofstream offset_data;
    ifstream myfile(filename);
    string line;
    double angle1, angle2, angle3, angle4, angle5, 
            angle6, angle7, angle8, angle9, angle10,
            angle11, angle12;

    offset_data.open("offset.txt");

    for (i = 0; i < count - 1; ++i)
    {
        getline(myfile, line);
    }
    while (getline(myfile, line))
    {   
        // cout << line << "\n";
        stringstream ss(line);
        ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10 >> angle11 >> angle12;
    }

    Eigen::RowVectorXd offset(10);
    double gear_ratio = 1.545;



    //  ctrl_space_calibration_pose_angle
    Eigen::RowVectorXd theta_c_0(10);
    Eigen::RowVectorXd theta_m_0(10);
    Eigen::RowVectorXd theta_m_raw_0(10);

    // theta_c_0 << 0.0, 0.0, 7, -96, 83, 0.0, 0.0, 7, -96, 83;

    theta_c_0 << 0.0, 0.0, 0, -90, 0, 0.0, 0.0, 0, -90, 0;
    theta_c_0 = theta_c_0 * M_PI / 180; // convert to radian
    std::cout<< "calibration pose in ctrl space is " << std::endl << theta_c_0 << std::endl;

    theta_m_0 = theta_c_0;
    theta_m_0(4) = (theta_c_0(4))+(theta_c_0(3));
    theta_m_0(9) = (theta_c_0(9))+(theta_c_0(8));
    theta_m_0(3) = theta_m_0(3)*gear_ratio;
    theta_m_0(8) = theta_m_0(8)*gear_ratio;
    theta_m_0(5) = -theta_m_0(5);
    theta_m_0(6) = -theta_m_0(6);
    theta_m_0(7) = -theta_m_0(7);

    theta_m_raw_0 << angle2, angle3, angle10, angle11, angle12, angle5, angle6, angle7, angle8, angle9; //already in radian

    offset = theta_m_0 - theta_m_raw_0;

    std::cout << offset << std::endl;
    offset_data << offset;

    std::cout << "Angle Calibration Successful" << std::endl;

    // offset_data.close();

}

int main(){
    angle_offset();
    // rpy_offset();
}