#include <cassert>

#include <pf/Log.h>

using namespace pf;

bool OdometryReading::operator==(const OdometryReading& other)
{
    return (this->x == other.x &&
            this->y == other.y &&
            this->theta == other.theta);
}

void SensorReading::print()
{
    if (!is_laser) {
        printf("\t\tO\t%f\t%f\t%f\t%f\n", robot_in_odom.x,
                                          robot_in_odom.y,
                                          robot_in_odom.theta,
                                          ts);
    } else {
        printf("\t\tL\t%f\t%f\t%f\n\t\t \t%f\t%f\t%f\n\t\t\t%f\n",
                                                    robot_in_odom.x,
                                                    robot_in_odom.y,
                                                    robot_in_odom.theta,
                                                    laser_in_odom.x,
                                                    laser_in_odom.y,
                                                    laser_in_odom.theta,
                                                    ts);
    }
}

Log::Log(std::string file_name)
    : iterator_(0)
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
    size_t num_odom_readings = 0;
    size_t num_laser_readings = 0;
    while (!feof(fin)) {
        char type_of_reading;
        if (fscanf(fin, "%c ", &type_of_reading) <=0) {
            printf("cannot determine type_of_reading\n");
            return false;
        }
        // If it is odometry
        if (type_of_reading == 'O') {
            // Read the values
            SensorReading odom_reading;
            if (fscanf(fin, "%lf %lf %lf %lf\n", &odom_reading.robot_in_odom.x,
                                                 &odom_reading.robot_in_odom.y,
                                                 &odom_reading.robot_in_odom.theta,
                                                 &odom_reading.ts) != 4) {
                printf("cannot parse odom_reading\n");
                return false;
            }
            odom_reading.is_laser = false;
            // add to our list of odometry readings
            sensor_data_.push_back(std::move(odom_reading));
            num_odom_readings++;
        }
        // If it is laser
        else if (type_of_reading == 'L') {
            // Read the values
            SensorReading laser_reading;
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
                laser_reading.scan_data.push_back(temp);
            }
            if (fscanf(fin, "%lf\n", &laser_reading.ts) <= 0) {
                printf("error parsing the timestamp on laser reading\n");
                return false;
            }
            laser_reading.is_laser = true;
            sensor_data_.push_back(std::move(laser_reading));
            num_laser_readings++;
        }
    }
    printf("Read : %lu odom_readings and %lu laser_readings\n",
                                            num_odom_readings,
                                            num_laser_readings);
    total_readings_ = num_laser_readings + num_odom_readings;
    return true;
}

SensorReading Log::getNextReading()
{
    assert(!isEmpty());
    return sensor_data_[iterator_++];
}