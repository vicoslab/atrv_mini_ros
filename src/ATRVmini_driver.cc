/*
 *  ATRVmini driver - Jaka Cikac 2013
 *  Modfied from: ATRVmini Driver - By David Lu!! 2/2010
 */

#include <ATRVmini_driver.h>
#include <ATRVmini_config.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace std;

ATRVmini::ATRVmini() {
    found_distance = false;
    bumps = new int*[2];

    for (int index=0;index<2;index++) {
        bumps[index] = new int[BUMPERS_PER[index]];
        for (int i=0;i<BUMPERS_PER[index];i++) {
            bumps[index][i] =0;
        }
    }

}

ATRVmini::~ATRVmini() {
    delete bumps[0];
    delete bumps[1];
    delete bumps;
}

float ATRVmini::getDistance() {
    if (!found_distance && isOdomReady()) {
        first_distance = distance;
        found_distance = true;
    }

    return (distance-first_distance) / (float) ODO_DISTANCE_CONVERSION;
}

float ATRVmini::getBearing() {
    return (bearing-HOME_BEARING) / (float) ODO_ANGLE_CONVERSION;
}

float ATRVmini::getTranslationalVelocity() const {
    return transVelocity / (float) ODO_DISTANCE_CONVERSION;
}

float ATRVmini::getRotationalVelocity() const {
    return rotVelocity / (float) ODO_ANGLE_CONVERSION;
}

float ATRVmini::getVoltage() const {
    if (voltage==0.0)
        return 0.0;
    else
        return voltage/100.0 + POWER_OFFSET;
}

float ATRVmini::getPercentage() const {

    float volt = getVoltage();

    // 12S voltage thresholds
    const float V_MAX = 25.77f;
    const float V_MID = 24.45f;
    const float V_MIN = 23.25f;

    if(volt > V_MAX)
        volt = V_MAX;
    else if(volt < V_MIN)
        volt = V_MIN;

    if (volt >= V_MID) {
        // Map 100% - 50% to 100% - 5%
        return 0.05f + (volt - V_MID) / (V_MAX - V_MID) * 0.95f;
    } else {
        // Map 50% - 0% to 5% - 0%
        return (volt - V_MIN) / (V_MID - V_MIN) * 0.5f;
    }
}

bool ATRVmini::isPluggedIn() const {
    return getVoltage() > PLUGGED_THRESHOLD;
}

int ATRVmini::getNumBodySonars() const {
    return SONARS_PER_RING[BODY_INDEX];
}

int ATRVmini::getNumBaseSonars() const {
    return SONARS_PER_RING[BASE_INDEX];
}

void ATRVmini::getBodySonarReadings(float* readings) const {
    getSonarReadings(BODY_INDEX, readings);
}

void ATRVmini::getBaseSonarReadings(float* readings) const {
    getSonarReadings(BASE_INDEX, readings);
}

void ATRVmini::getBodySonarPoints(sensor_msgs::msg::PointCloud* cloud) const {
    getSonarPoints(BODY_INDEX, cloud);
}
void ATRVmini::getBaseSonarPoints(sensor_msgs::msg::PointCloud* cloud) const {
    getSonarPoints(BASE_INDEX, cloud);
}

int ATRVmini::getBodyBumps(sensor_msgs::msg::PointCloud* cloud) const {
    return getBumps(BODY_INDEX, cloud);
}

int ATRVmini::getBaseBumps(sensor_msgs::msg::PointCloud* cloud) const {
    return getBumps(BASE_INDEX, cloud);
}

void ATRVmini::setSonarPower(bool on) {
    unsigned long echo, ping, set, val;
    if (on) {
        echo = SONAR_ECHO_DELAY;
        ping = SONAR_PING_DELAY;
        set = SONAR_SET_DELAY;
        val = 2;
    } else {
        echo = ping = set = val = 0;
    }
    configureSonar(echo, ping, set, val);
}

void ATRVmini::setMovement(float tvel, float rvel, float acceleration) {
    setVelocity(tvel * ODO_DISTANCE_CONVERSION,
                rvel * ODO_ANGLE_CONVERSION,
                acceleration * ODO_DISTANCE_CONVERSION);
}


void ATRVmini::getSonarReadings(const int ringi, float* adjusted_ranges) const {
    int i = 0;
    for (int x = SONAR_RING_BANK_BOUND[ringi];x<SONAR_RING_BANK_BOUND[ringi+1];x++) {
        for (int y=0;y<SONARS_PER_BANK[x];y++) {
            int range = sonar_ranges[x*SONAR_MAX_PER_BANK+y];
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            float fRange = range / (float) RANGE_CONVERSION;
            adjusted_ranges[i] = fRange;
            i++;
        }
    }
}

void ATRVmini::getSonarPoints(const int ringi, sensor_msgs::msg::PointCloud* cloud) const {
    int numSonar = SONARS_PER_RING[ringi];
    float* readings = new float[numSonar];
    getSonarReadings(ringi, readings);
    cloud->points.resize(numSonar);
    int c = 0;
    for (int i = 0; i < numSonar; ++i) {
        if (readings[i] < SONAR_MAX_RANGE/ (float) RANGE_CONVERSION) {
            double angle =  SONAR_RING_START_ANGLE[ringi] + SONAR_RING_ANGLE_INC[ringi]*i;
            angle *= M_PI / 180.0;

            double d = SONAR_RING_DIAMETER[ringi] + readings[i];
            cloud->points[c].x = cos(angle)*d;
            cloud->points[c].y = sin(angle)*d;
            cloud->points[c].z = SONAR_RING_HEIGHT[ringi];
            c++;
        }
    }
}

int ATRVmini::getBumps(const int index, sensor_msgs::msg::PointCloud* cloud) const {
    int c = 0;
    double wedge = 2 * M_PI / BUMPERS_PER[index];
    double d = SONAR_RING_DIAMETER[index]*1.1;
    int total = 0;
    for (int i=0;i<BUMPERS_PER[index];i++) {
        int value = bumps[index][i];
        for (int j=0;j<4;j++) {
            int mask = 1 << j;
            if ((value & mask) > 0) {
                total++;
            }
        }
    }

    cloud->points.resize(total);
    if (total==0)
        return 0;
    for (int i=0;i<BUMPERS_PER[index];i++) {
        int value = bumps[index][i];
        double angle = wedge * (2.5 - i);
        for (int j=0;j<4;j++) {
            int mask = 1 << j;
            if ((value & mask) > 0) {
                double aoff = BUMPER_ANGLE_OFFSET[j]*wedge/3;
                cloud->points[c].x = cos(angle-aoff)*d;
                cloud->points[c].y = sin(angle-aoff)*d;
                cloud->points[c].z = BUMPER_HEIGHT_OFFSET[index][j];
                c++;
            }
        }
    }
    return total;

}

void ATRVmini::processDioEvent(unsigned char address, unsigned short data) {

    if (address == HEADING_HOME_ADDRESS) {
        home_bearing = bearing;
        //printf("ATRVmini Home %f \n", home_bearing / (float) ODO_ANGLE_CONVERSION);
    }   // check if the dio packet came from a bumper packet
    else if ((address >= BUMPER_ADDRESS) && (address < (BUMPER_ADDRESS+BUMPER_COUNT))) {
        int index =0, rot = address - BUMPER_ADDRESS;
        if (rot > BUMPERS_PER[index]) {
            rot -= BUMPERS_PER[index];
            index++;
        }
        bumps[index][rot] = data;
    } else {
        //printf("ATRVmini DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
    }
}


