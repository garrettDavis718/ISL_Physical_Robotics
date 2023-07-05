#Creating a object class for closest objects to reference

class Object():
    def __init__(self, id, location, distance, is_green, lidar_angles):
        self.id = id
        self.location = location
        self.distance = round(distance, 5)
        self.is_green = is_green
        self.lidar_angles = None

    def __str__(self):
        return f"id - {self.id}, loc - {self.location}, dist - {self.distance}, isGreen - {self.is_green}, angles - {self.lidar_angles}"
    
    def object_to_tuple(self):
        return (self.id, self.location, self.distance, self.is_green, self.lidar_angles)