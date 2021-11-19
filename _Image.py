

class _Image:

    drone_model = ""

    def __init__(self, pitch_angle, image_width, image_height, focal_length, horizontal_angle, altitude, filename,
                 drone_model, latitude, longitude):
        self.pitch_angle = pitch_angle
        self.image_width = image_width
        self.image_height = image_height
        self.focal_length = focal_length
        self.horizontal_angle = horizontal_angle
        self.altitude = altitude
        _Image.drone_model = drone_model
        self.filename = filename
        self.latitude = latitude
        self.longitude = longitude

    def get_pitch_angle(self):
        return self.pitch_angle

    def get_image_width(self):
        return self.image_width

    def get_image_height(self):
        return self.image_height

    def get_focal_length(self):
        return self.focal_length

    def get_horizontal_angle(self):
        return self.horizontal_angle

    def get_altitude(self):
        return self.altitude

    def get_filename(self):
        return self.filename

    @staticmethod
    def get_drone_model():
        return _Image.drone_model

    def get_latitude(self):
        return self.latitude

    def get_longitude(self):
        return self.longitude
