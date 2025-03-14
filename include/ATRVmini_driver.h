#ifndef ATRVmini_DRIVER_H
#define ATRVmini_DRIVER_H

#include <rflex_driver.h>
#include <sensor_msgs/msg/point_cloud.hpp>

/**
 *  ATRVmini Driver - Jaka Cikac 2013
 *  Modified from B21 Driver - By David Lu 2/2010
 */
class ATRVmini : public RFLEX {
    public:
        ATRVmini();
        virtual ~ATRVmini();
        void setSonarPower(bool);
        float getDistance();
        float getBearing();
        float getTranslationalVelocity() const;
        float getRotationalVelocity() const;
        float getVoltage() const;
        float getPercentage() const;
        bool isPluggedIn() const;
        int getNumBodySonars() const;
        int getNumBaseSonars() const;

        /** Get readings from the sonar on the body of the ATRVmini
         * in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getBodySonarReadings(float* readings) const;
        /** Get readings from the sonar on the base of the ATRVmini
         * in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getBaseSonarReadings(float* readings) const;

        /** Gets a point cloud for sonar readings from body
         * \param cloud Data structure into which the sonar readings are saved */
        void getBodySonarPoints(sensor_msgs::msg::PointCloud* cloud) const;

        /** Gets a point cloud for sonar readings from base
         * \param cloud Data structure into which the sonar readings are saved */
        void getBaseSonarPoints(sensor_msgs::msg::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors on the body
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors
         */
        int getBodyBumps(sensor_msgs::msg::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors on the base
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors */
        int getBaseBumps(sensor_msgs::msg::PointCloud* cloud) const;

        /** Sets the motion of the robot
         * \param tvel Translational velocity (in m/s)
         * \param rvel Rotational velocity (in radian/s)
         * \param acceleration Translational acceleration (in m/s/s) */
        void setMovement(float tvel, float rvel, float acceleration);

        /** Processes the DIO packets - called from RFflex Driver
         * \param address origin
         * \param data values */
        void processDioEvent(unsigned char address, unsigned short data);

        /** Detects whether the robot has all the necessary components
         * to calculate odometry
         * \return bool true if robot has read its distance, bearing and home bearing */
        bool isOdomReady() const {
            return odomReady==3;
        }

    private:
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param readings Data structure into which the sonar readings are saved */
        void getSonarReadings(const int ringi, float* readings) const;
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param cloud Data structure into which the sonar readings are saved */
        void getSonarPoints(const int ringi, sensor_msgs::msg::PointCloud* cloud) const;

        /**\param index BODY_INDEX or BASE_INDEX
           \param cloud Data structure into which the bump sensors are saved
           \return number of active bump sensors
        */
        int getBumps(const int index, sensor_msgs::msg::PointCloud* cloud) const;

        int first_distance;
        bool found_distance;
        int home_bearing; ///< Last home bearing (arbitrary units)
        int** bumps;

        // Not allowed to use these
        ATRVmini(const ATRVmini &ATRVmini); 				///< Private constructor - Don't use
        ATRVmini &operator=(const ATRVmini &ATRVmini); 	///< Private constructor - Don't use
};

#endif

