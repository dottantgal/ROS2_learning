#  *
#  * @file vehicle_plugins.py
#  *
#  * @brief Definition of the derived classes that will be used as plugins
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

from vehicle_base_py import RegularVehicle


class Motorbike(RegularVehicle):
    """Motorbike plugin implementation."""

    def __init__(self):
        self.vehicle_type_ = None

    def initialize(self, vehicle_type: str) -> None:
        """Initialize the motorbike."""
        self.vehicle_type_ = vehicle_type

    def set_num_passengers(self) -> int:
        """Return the number of passengers for a motorbike."""
        return 2

    def get_vehicle_type(self) -> str:
        """Return the vehicle type."""
        return self.vehicle_type_


class Bicycle(RegularVehicle):
    """Bicycle plugin implementation."""

    def __init__(self):
        self.vehicle_type_ = None

    def initialize(self, vehicle_type: str) -> None:
        """Initialize the bicycle."""
        self.vehicle_type_ = vehicle_type

    def set_num_passengers(self) -> int:
        """Return the number of passengers for a bicycle."""
        return 1

    def get_vehicle_type(self) -> str:
        """Return the vehicle type."""
        return self.vehicle_type_


class Truck(RegularVehicle):
    """Truck plugin implementation."""

    def __init__(self):
        self.vehicle_type_ = None

    def initialize(self, vehicle_type: str) -> None:
        """Initialize the truck."""
        self.vehicle_type_ = vehicle_type

    def set_num_passengers(self) -> int:
        """Return the number of passengers for a truck."""
        return 5

    def get_vehicle_type(self) -> str:
        """Return the vehicle type."""
        return self.vehicle_type_


# Entry points for plugin discovery
# These functions will be registered as entry points in setup.py
def create_motorbike():
    """Factory function to create a Motorbike instance."""
    return Motorbike()


def create_bicycle():
    """Factory function to create a Bicycle instance."""
    return Bicycle()


def create_truck():
    """Factory function to create a Truck instance."""
    return Truck()

