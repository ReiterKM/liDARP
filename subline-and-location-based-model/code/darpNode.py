from util import *
from typing import Optional

class DarpNode:
    def __init__(self, darp_station: int, bus_station: Optional[int], corresponding_request: Optional[int], type: str, direction = Optional[str]) -> None:
        self._validate_type(type)

        if not (darp_station is None) and not isinstance(darp_station, int):
            raise TypeError("Bus station of Location-Based node must be an integer.")

        self.darp_station = darp_station
        self.bus_station = bus_station
        self.request = corresponding_request
        self.direction = direction
        self.e = None
        self.l = None
        self.type = type
        self.name = None
        self.has_arrival_service_promise = False
        self.has_departure_service_promise = False

    def _validate_type(self, type):
        if type not in ["o", "d", "o_bar", "d_bar", "start_depot", "end_depot"]:
            raise ValueError("DARP Node type of {} was not recognized.".format(self.type))
        
    def is_depot(self):
        return (self.type in ["start_depot", "end_depot"])
        
    def __eq__(self, other):
        if isinstance(other, DarpNode):
            return self.darp_station == other.darp_station
        return False
    
    def __lt__(self, other):
        if isinstance(other, DarpNode):
            return self.darp_station < other.darp_station
        return False
    
    def __gt__(self, other):
        if isinstance(other, DarpNode):
            return self.darp_station > other.darp_station
        return False
    
    def __sub__(self, other):
        return self.darp_station - other.darp_station
        