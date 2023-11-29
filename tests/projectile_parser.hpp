#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

// Projectile struct
struct Projectile {
    int id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};


bool checkInputErrors(std::stringstream& ss, char expectedDelimiter, const std::string& errorMessage) {
    char delimiter;
    if (!(ss >> delimiter) || delimiter != expectedDelimiter) {
        std::cerr << "Error: " << errorMessage << std::endl;
        return true;
    }
    return false;
}

// Function to parse the input string
std::pair<double, std::vector<Projectile>> parseInput(const std::string& input) {
    std::stringstream ss(input);

    double time;
    if (!(ss >> time) || checkInputErrors(ss, ';', "Invalid time format")) {
        return {-1.0, {}};
    }

    std::vector<Projectile> projectiles;
    while (ss.peek() > 0) {
        Projectile proj;
        
        if (!(ss >> proj.id) || checkInputErrors(ss, ',', "Invalid id format")) {
            return {-1.0, {}};
        }

        // Read position
        if (!(ss >> proj.position.x()) || checkInputErrors(ss, ',', "Invalid position format") ||
            !(ss >> proj.position.y()) || checkInputErrors(ss, ',', "Invalid position format") ||
            !(ss >> proj.position.z())) {
            return {-1.0, {}};
        }

        // Read velocity
        if (checkInputErrors(ss, ',', "Expected ','") ||
            !(ss >> proj.velocity.x()) || checkInputErrors(ss, ',', "Invalid velocity format") ||
            !(ss >> proj.velocity.y()) || checkInputErrors(ss, ',', "Invalid velocity format") ||
            !(ss >> proj.velocity.z())) {
            return {-1.0, {}};
        }
        if (checkInputErrors(ss, ';', "Expected ';'")) {
            return {-1.0, {}};
        }

        projectiles.push_back(proj);
    }

    return {time, projectiles};
}