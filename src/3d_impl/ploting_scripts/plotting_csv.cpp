// plotting_csv.cpp

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include "../forward_kinematics/forward_kinematics_roarm_double.h"

namespace fs = std::filesystem;

using Configuration = std::vector<double>;

// --- Helper to call ForwardKinematics using a vector ---
auto call_forward_kinematics(const Configuration& joint_angles) {
    if (joint_angles.size() != 5) {
        throw std::runtime_error("call_forward_kinematics error: Expected 5 joint angles.");
    }

    double qcos[5];
    double qsin[5];
    double v[5] = {0};
    double a[5] = {0};

    for (size_t i = 0; i < 5; ++i) {
        qcos[i] = std::cos(joint_angles[i]);
        qsin[i] = std::sin(joint_angles[i]);
    }

    return ForwardKinematics(
        qcos[0], qcos[1], qcos[2], qcos[3], qcos[4],
        qsin[0], qsin[1], qsin[2], qsin[3], qsin[4],
        v[0], v[1], v[2], v[3], v[4],
        a[0], a[1], a[2], a[3], a[4]
    );
}

// --- Main program ---
int main() {
    std::string input_file;
    std::cout << "Enter the path to the RRT CSV file: ";
    std::cin >> input_file;

    if (!fs::exists(input_file)) {
        std::cerr << "Error: File " << input_file << " does not exist.\n";
        return 1;
    }

    std::ifstream infile(input_file);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << input_file << " for reading.\n";
        return 1;
    }

    std::string output_file = "fk_output.csv";
    std::ofstream outfile(output_file);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file for writing.\n";
        return 1;
    }

    outfile << "section,x,y,z\n";

    std::string line;
    std::string current_section = "";

    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') {
            if (line.find("Start") != std::string::npos) current_section = "Start";
            else if (line.find("Goal_Path") != std::string::npos || line.find("Simplified_Path") != std::string::npos) current_section = "Path";
            else if (line.find("Goal") != std::string::npos) current_section = "Goal";
            else if (line.find("Obstacles") != std::string::npos) current_section = "Obstacle";
            continue;
        }

        std::stringstream ss(line);
        std::vector<double> values;
        std::string value;

        while (std::getline(ss, value, ',')) {
            try {
                values.push_back(std::stod(value));
            } catch (const std::invalid_argument&) {
                // If not a number, ignore this line
                values.clear();
                break;
            }
        }
        
        if (current_section == "Obstacle" && values.size() == 4) {
            outfile << "Obstacle," << values[0] << "," << values[1] << "," << values[2] << "\n";
        }
        else if ((current_section == "Start" || current_section == "Goal" || current_section == "Path") && (values.size() == 5 || values.size() == 10)) {
            if (values.size() == 5) {
                auto fk_result = call_forward_kinematics(values);
                for (const auto& frame : fk_result.SE3) {
                    outfile << current_section << "," << frame.translation[0] << "," << frame.translation[1] << "," << frame.translation[2] << "\n";
                }
            } else if (values.size() == 10) {
                Configuration src(values.begin(), values.begin() + 5);
                Configuration dst(values.begin() + 5, values.end());

                auto fk_src = call_forward_kinematics(src);
                auto fk_dst = call_forward_kinematics(dst);

                for (const auto& frame : fk_src.SE3) {
                    outfile << current_section << "," << frame.translation[0] << "," << frame.translation[1] << "," << frame.translation[2] << "\n";
                }
                for (const auto& frame : fk_dst.SE3) {
                    outfile << current_section << "," << frame.translation[0] << "," << frame.translation[1] << "," << frame.translation[2] << "\n";
                }
            }
        }
    }

    infile.close();
    outfile.close();

    std::cout << "CSV created: " << output_file << "\n";

    return 0;
}