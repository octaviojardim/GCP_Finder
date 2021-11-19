
class _GroundControlPoint:

    format_ = ""

    def __init__(self, id_, lat, long, alt, format_):
        self.id_ = id_
        self.lat = lat
        self.long = long
        self.alt = alt
        _GroundControlPoint.format_ = format_

    def get_id(self):
        return self.id_

    def get_lat(self):
        return self.lat

    def get_long(self):
        return self.long

    def get_alt(self):
        return self.alt

    @staticmethod
    def get_format_():
        return _GroundControlPoint.format_

