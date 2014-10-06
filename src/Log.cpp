#include <stdexcept>

#include <pf/Log.h>

using namespace pf;

Log::Log(std::string file_name)
{
    if (!loadFromFile(file_name))
        throw std::runtime_error("Could not initialize log!\n");
}

bool Log::loadFromFile(std::string file_name)
{
    FILE* fin = fopen(file_name.c_str(), "r");
    if (!fin) {
        printf("could not find file (%s)\n", file_name.c_str());
        return false;
    }
    while (!feof(fin)) {
        char type_of_reading;
        if (fscanf(fin, "%c ", &type_of_reading) <=0) {
            printf("cannot determine type_of_reading\n");
            return false;
        }
        // If it is odometry
        if (type_of_reading == 'O') {
            // Read the values
            OdometryReading odom_reading;
            if (fscanf(fin, "%lf %lf %lf %lf\n", &odom_reading.x,
                                                 &odom_reading.y,
                                                 &odom_reading.theta,
                                                 &odom_reading.ts) != 4) {
                printf("cannot parse odom_reading\n");
                return false;
            }
            // add to our list of odometry readings
            odom_readings_.push_back(std::move(odom_reading));
        }
        // If it is laser
        else if (type_of_reading == 'L') {
            // Read the values
            LaserReading laser_reading;
            if (fscanf(fin, "%lf %lf %lf ", &laser_reading.robot_in_odom.x,
                                             &laser_reading.robot_in_odom.y,
                                             &laser_reading.robot_in_odom.theta)
                                                                        != 3)
            {
                printf("cannot parse laser reading robot_in_odom\n");
                return false;
            }
            if (fscanf(fin, "%lf %lf %lf ", &laser_reading.laser_in_odom.x,
                                            &laser_reading.laser_in_odom.y,
                                            &laser_reading.laser_in_odom.theta)
                                                                        != 3) {
                printf("cannot parse laser reading laser_in_odom\n");
                return false;
            }
            for (int i = 0; i < 180; ++i) {
                int temp;
                if (fscanf(fin, "%d ", &temp) <=0 ){
                    printf("error parsing %d-th pixel reading\n", i);
                    return false;
                }
                laser_reading.readings.push_back(temp);
            }
            if (fscanf(fin, "%lf\n", &laser_reading.ts) <= 0) {
                printf("error parsing the timestamp on laser reading\n");
                return false;
            }
            laser_readings_.push_back(std::move(laser_reading));
        }
    }
    printf("Read : %lu odom_readings and %lu laser_readings\n",
                                            odom_readings_.size(),
                                            laser_readings_.size());
    return true;
}