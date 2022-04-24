/**
 * @file vehicle_plugins.cpp
 *
 * @brief Definition of the derived class that will be used as plugin
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "vehicle_base/regular_vehicle.hpp"

// derived classes declaration which will be instanciated through pluginlib
namespace vehicle_plugins
{
  class Motorbike : public vehicle_base::RegularVehicle
  {
    public:
      void initialize(std::string vehicleType) override
      {
        vehicleType_ = vehicleType;
      }

      int setNumPassengers() override
      {
        return 2;
      }

      std::string getVehicleType() override
      {
        return vehicleType_;
      }

    protected:
      std::string vehicleType_;
  };

  class Bicycle : public vehicle_base::RegularVehicle
  {
    public:
      void initialize(std::string vehicleType) override
      {
        vehicleType_ = vehicleType;
      }

      int setNumPassengers() override
      {
        return 1;
      }

      std::string getVehicleType() override
      {
        return vehicleType_;
      }

    protected:
      std::string vehicleType_;
  };

  class Truck : public vehicle_base::RegularVehicle
  {
    public:
      void initialize(std::string vehicleType) override
      {
        vehicleType_ = vehicleType;
      }

      int setNumPassengers() override
      {
        return 5;
      }

      std::string getVehicleType() override
      {
        return vehicleType_;
      }

    protected:
      std::string vehicleType_;
  };
}

#include <pluginlib/class_list_macros.hpp>

// registration of the derived classes as plugin
PLUGINLIB_EXPORT_CLASS(vehicle_plugins::Motorbike, vehicle_base::RegularVehicle)
PLUGINLIB_EXPORT_CLASS(vehicle_plugins::Bicycle, vehicle_base::RegularVehicle)
PLUGINLIB_EXPORT_CLASS(vehicle_plugins::Truck, vehicle_base::RegularVehicle)