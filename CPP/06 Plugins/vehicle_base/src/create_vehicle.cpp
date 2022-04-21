/**
 * @file create_vehicle.cpp
 *
 * @brief Creation of derived objects through the available plugin
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include <pluginlib/class_loader.hpp>
#include "vehicle_base/regular_vehicle.hpp"
#include <iostream>

int main(int argc, char** argv)
{
  // No parameters passed to the main (used to do initializing the node)
  // these two lines avoid compile warnings
  (void) argc; (void) argv;

  pluginlib::ClassLoader<vehicle_base::RegularVehicle> VehiclePluginLoader(
  "vehicle_base", "vehicle_base::RegularVehicle");

 try
  {
    // the istance of the derived class Motorbike is created through the plugin as shared ptr
    // and it's called the initialize method setting the vehicle type to "motorbike"
    std::shared_ptr<vehicle_base::RegularVehicle> motorbike = VehiclePluginLoader.createUniqueInstance(
        "vehicle_plugins::Motorbike");
    motorbike->initialize("motorbike");

    // same as motrobike but with bicycle
    std::shared_ptr<vehicle_base::RegularVehicle> bicycle = VehiclePluginLoader.createUniqueInstance(
        "vehicle_plugins::Bicycle");
    bicycle->initialize("bicycle");

    // use of the method getVehicleType through the shared ptrs of the instances derived
    // from the base class using the plugin
    std::cout << "First vehicle type is -> " << motorbike->getVehicleType() << std::endl;
    std::cout << "Second vehicle type is -> " << bicycle->getVehicleType() << std::endl;
  }
  catch(pluginlib::PluginlibException& ex)
  {
    std::cout << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

   try
  {
    // here is trying to initialize a shared ptr of a derived class which is not available with
    // the created plugin, so it gives an error
    std::shared_ptr<vehicle_base::RegularVehicle> rocket = VehiclePluginLoader.createUniqueInstance("vehicle_plugins::Rocket");
    rocket->initialize("rocket");

    std::cout << "Third vehicle type is -> " << rocket->getVehicleType() << std::endl;
  }
  catch(pluginlib::PluginlibException& ex)
  {
    std::cout << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  return 0;
}