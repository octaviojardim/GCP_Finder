import json


class _Statistics:
    total_images = 0
    gcp_found = 0  # number of arUco markers found
    img_with_gcp = 0  # number of images that probably contains a GCP

    def __init__(self, total_images):
        self.total_images = total_images
        self.init()

    def save_statistic(self, n, stage):

        f = open('statistic.json', "r")
        data = json.load(f)
        f.close()

        if stage == "meta":
            data["Processed"] += n
            data["Total"] = self.total_images

        elif stage == "contains_gcp":
            self.img_with_gcp += n
            data["Contains_GCP"] += n

        elif stage == "aruco":
            data["Processed_aruco"] += n

        elif stage == "gcp_found":
            self.gcp_found += n
            data["GCP_found"] = self.gcp_found

        f = open('statistic.json', "w")
        json.dump(data, f)
        f.close()

    @staticmethod
    def init():
        f = open('statistic.json', "r")
        data = json.load(f)
        f.close()
        data["Processed"] = 0
        data["Total"] = 0
        data["Contains_GCP"] = 0
        data["Processed_aruco"] = 0
        data["Total_aruco"] = 0
        data["GCP_found"] = 0
        f = open('statistic.json', "w")
        json.dump(data, f)
        f.close()

    @staticmethod
    def update_aruco(value):
        f = open('statistic.json', "r")
        data = json.load(f)
        f.close()
        data["Total_aruco"] = value
        f = open('statistic.json', "w")
        json.dump(data, f)
        f.close()

    @staticmethod
    def get_total_images():
        return _Statistics.total_images

    @staticmethod
    def get_gcp_found():
        return _Statistics.gcp_found

    @staticmethod
    def get_img_with_gcp():
        return _Statistics.img_with_gcp
