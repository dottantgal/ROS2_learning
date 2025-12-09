#  *
#  * @file regular_vehicle.py
#  *
#  * @brief Definition of the base class which is derived through plugin
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

from abc import ABC, abstractmethod


class RegularVehicle(ABC):
    """
    Base abstract class for vehicle plugins.
    
    This method "initialize" is necessary to init an object of this
    class due to the fact that Python entry points don't allow constructors 
    with parameters easily.
    """

    @abstractmethod
    def initialize(self, vehicle_type: str) -> None:
        """Initialize the vehicle with a type."""
        pass

    @abstractmethod
    def set_num_passengers(self) -> int:
        """Return the number of passengers."""
        pass

    @abstractmethod
    def get_vehicle_type(self) -> str:
        """Return the vehicle type."""
        pass

