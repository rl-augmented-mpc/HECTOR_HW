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

    Eigen::RowVectorXd corrected_angle(10);
    Eigen::RowVectorXd init_angle(10);
    Eigen::RowVectorXd corrected_angle_2(10);
    Eigen::RowVectorXd offset(10);

    double gear_ratio = 1.545;
    double knee_joint_pos = -1.5708 * gear_ratio;


    // corrected_angle << -0.02152, 0.03086, -0.04939, -2.63111, 1.44497, -0.01307, -0.01974, -0.1187, -2.59689, 1.40156;
    corrected_angle << 0.0, 0.0, 0.0, knee_joint_pos, 0.0, 0.0, 0.0, 0.0, knee_joint_pos, 0.0;
    init_angle << angle2, angle3, angle10, angle11, angle12, angle5, angle6, angle7, angle8, angle9;

    corrected_angle_2 = corrected_angle;

    corrected_angle_2(4) = (corrected_angle(4))+(corrected_angle(3)/gear_ratio);
    corrected_angle_2(9) = (corrected_angle(9))+(corrected_angle(8)/gear_ratio);

    corrected_angle_2(3) = corrected_angle_2(3);
    corrected_angle_2(8) = corrected_angle_2(8);

    corrected_angle_2(5) = -corrected_angle_2(5);
    corrected_angle_2(6) = -corrected_angle_2(6);
    corrected_angle_2(7) = -corrected_angle_2(7);



    offset = corrected_angle_2 - init_angle;

    std::cout << offset << std::endl;
    offset_data << offset;

    std::cout << "Angle Calibration Successful" << std::endl;

    // offset_data.close();

}

int main(){
    angle_offset();
    // rpy_offset();
}