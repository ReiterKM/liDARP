#include "DARPH.h"

// note: need to re-name this function to main if you want to run the model with Q = 6
int main_6_veh(int argc, char* argv[])
{
    bool accept_all = false;
    bool consider_shortcuts = false;
    bool consider_excess_ride_time = false;
    bool dynamic = false; // liDARP is currently only implemented for the static case
    bool heuristic = true;
    int time_limit = 60 * 60; // cplex time limit in seconds

    std::string data_directory(argv[1]);
    int instance_mode(std::stoi(argv[2]));
    std::string location_file_name;

    std::cout << "Instance mode: " << instance_mode << std::endl;

    if (instance_mode == 2 || instance_mode == 3) {
        if (argc != 4) {
            throw std::invalid_argument("Not enough arguments were passed to the program.");
        }
        location_file_name = argv[3];
        std::string location_file_path = data_directory + location_file_name + ".txt";
        if (!std::filesystem::exists(location_file_path)) {
            std::cerr << "Additional file does not exist." << endl;
            throw std::invalid_argument("Invalid file.");
        }
    }

    using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;
    for (const auto& path : recursive_directory_iterator(data_directory))
    {
        if (std::filesystem::is_regular_file(path) && path.path().extension() == ".txt") {
            std::string path_to_instance = path.path().string();
            std::string instance = path.path().stem().string();

            if (instance_mode == 2 || instance_mode == 3) {
                if (instance == location_file_name) {
                    continue;
                }
            }

            std::cout << "Reading file: " << path << std::endl;

            const std::array<double, 2>& w = { 10, 1 }; // weights (accepted pax, saved distance)

            auto [num_requests, veh_capacity] = DARPGetDimension(path_to_instance);

            if (veh_capacity != 6) {
                report_error("Vehicle capacity of %d does not match chosen code.", veh_capacity);
            }
            auto D = DARP(num_requests);
            auto RH = RollingHorizon<6>(num_requests);

            // switch between different types of instances
            // 1: request file, all distances = 1
            // 2: request and station file
            // 3: request file and distance matrix
            if (instance_mode < 0 || instance_mode > 3) {
                throw std::domain_error("Instance mode unknown.");
            }
            D.set_instance_mode(instance_mode);

            // read instance from file
            D.read_file(path_to_instance, data_directory, instance);

            switch (instance_mode) {
            case 1:
                D.generate_distances();
                break;
            case 2:
                D.generate_distances(path_to_instance, data_directory, location_file_name, consider_shortcuts);
                break;
            case 3:
                D.generate_distances(path_to_instance, data_directory, location_file_name);
                break;
            default:
                break;
            }
            D.set_time_windows();
            D.transform_dynamic(); // save times of show-up to become_known_array

            auto G = DARPGraph<6>(num_requests);

            // solve instance
            std::array<double, 8> lsg = RH.solve(accept_all, consider_excess_ride_time, dynamic, heuristic, time_limit, D, G, w);

            RH.compute_stats(D);

            RH.detailed_file(D, instance, lsg);
        } // end if file
    } // end iteration
    return 0;
}