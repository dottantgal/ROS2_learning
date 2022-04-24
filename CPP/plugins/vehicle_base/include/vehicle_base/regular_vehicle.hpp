/**
 * @file regular_vehicle.cpp
 *
 * @brief Definition of the base class which is derived through plugin
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#ifndef VEHICLE_BASE_REGULAR_VEHICLE_HPP
#define VEHICLE_BASE_REGULAR_VEHICLE_HPP

#include <iostream>

// base abstract class
namespace vehicle_base
{
  class RegularVehicle
  {
    public:
      // this method "initialize" is necessary to init an object of this
      // class due to the fact that plugin lib doesn't allow constructors 
      // with parameters 
      virtual void initialize(std::string vehicleType) = 0;
      virtual int setNumPassengers() = 0;
      virtual std::string getVehicleType() = 0;
      virtual ~RegularVehicle(){}

    protected:
      RegularVehicle(){}
  };
}

#endif  // END VEHICLE_BASE_REGULAR_VEHICLE_HPP