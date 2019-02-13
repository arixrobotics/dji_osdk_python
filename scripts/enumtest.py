from enum import Enum

class flight_status_enum(Enum):
    M100_STATUS_ON_GROUND = 1
    M100_STATUS_TAKINGOFF = 2 
    M100_STATUS_IN_AIR = 3
    M100_STATUS_LANDING = 4 
    M100_STATUS_FINISHED_LANDING = 5


print(flight_status_enum.M100_STATUS_LANDING.value)
print(flight_status_enum(1))