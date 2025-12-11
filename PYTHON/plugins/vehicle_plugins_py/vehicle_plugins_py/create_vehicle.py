#  *
#  * @file create_vehicle.py
#  *
#  * @brief Creation of derived objects through the available plugins
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import sys
try:
    from pkg_resources import iter_entry_points
except ImportError:
    # For newer Python versions, use importlib.metadata
    try:
        from importlib.metadata import entry_points
        def iter_entry_points(group):
            return entry_points(group=group)
    except ImportError:
        # Fallback for older Python
        from importlib_metadata import entry_points
        def iter_entry_points(group):
            return entry_points(group=group)

from vehicle_base_py import RegularVehicle


def load_plugin(plugin_name: str) -> RegularVehicle:
    """
    Load a plugin by name using entry points.
    
    Args:
        plugin_name: Name of the plugin (e.g., 'motorbike', 'bicycle', 'truck')
    
    Returns:
        An instance of RegularVehicle
    
    Raises:
        ValueError: If the plugin is not found
    """
    # Load plugin using entry points
    for entry_point in iter_entry_points('vehicle_base_py.plugins', plugin_name):
        factory = entry_point.load()
        return factory()
    raise ValueError(f"Plugin '{plugin_name}' not found")


def main():
    """Main function to demonstrate plugin usage."""
    print("=== Loading Available Plugins ===")
    try:
        # Create instances of derived classes through the plugin system
        # The instance of the derived class Motorbike is created through the plugin
        # and it's called the initialize method setting the vehicle type to "motorbike"
        motorbike = load_plugin('motorbike')
        motorbike.initialize("motorbike")

        # Same as motorbike but with bicycle
        bicycle = load_plugin('bicycle')
        bicycle.initialize("bicycle")

        # Use of the method get_vehicle_type through the instances derived
        # from the base class using the plugin
        print(f"First vehicle type is -> {motorbike.get_vehicle_type()}")
        print(f"Second vehicle type is -> {bicycle.get_vehicle_type()}")
        print("✓ Successfully loaded Motorbike and Bicycle plugins")
    except Exception as ex:
        print(f"ERROR: The plugin failed to load. Error: {ex}")
        return 1

    print("\n=== Demonstrating Error Handling (Intentional) ===")
    print("Attempting to load non-existent 'Rocket' plugin...")
    try:
        # Here is trying to initialize a derived class which is not available with
        # the created plugin, so it gives an error - this is intentional to demonstrate error handling
        rocket = load_plugin('rocket')
        rocket.initialize("rocket")
        print(f"Third vehicle type is -> {rocket.get_vehicle_type()}")
    except Exception as ex:
        print(f"Expected error caught: {ex}")
        print("✓ Error handling works correctly - plugin system properly rejects non-existent plugins")

    return 0


if __name__ == '__main__':
    sys.exit(main())

