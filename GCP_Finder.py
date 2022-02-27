import os
import cv2
import sys
import math
import json
import time
import glob
import shutil
import exiftool
import geopy.distance
from cv2 import aruco
from pygeodesy.sphericalNvector import LatLon
from Image_ import Image_
from GroundControlPoint import GroundControlPoint
from Statistics import Statistics


class GCPFinder:
    found = "Ponto de Controle encontrado"
    not_found = "Ponto de Controle NAO encontrado na imagem"
    keywords = ["EXIF:Model", "MakerNotes:Yaw", "MakerNotes:CameraPitch", "XMP:RelativeAltitude", "File:ImageWidth",
                "File:ImageHeight", "EXIF:FocalLength", "EXIF:GPSLatitude", "EXIF:GPSLongitude", "EXIF"
                                                                                                 ":GPSLatitudeRef",
                "EXIF:GPSLongitudeRef"]

    def __init__(self):

        self.save_gcp_path = os.path.dirname(os.path.abspath("GCP_Finder.py"))
        self.total_images = 0
        self.SENSOR_WIDTH = 0
        self.lista_de_GCP_fixos = {}
        self.images_with_gcp = []
        self.image_list = []
        self.missing = False
        self.save_images = 0

        if len(sys.argv) < 4:
            print(
                'Usage: python GCP_Finder.py images_source_path coordinates_source_path border | [OPTIONAL] save_images_path')
            sys.exit(1)
        if len(sys.argv) == 5:
            self.save_images = 1
            # add '/' in the end of path if it's not present
            self.save_images_path = sys.argv[4] + "/" if sys.argv[4][-1] != "/" else sys.argv[4]

        self.images_source_path = sys.argv[1] + "/" if sys.argv[1][-1] != "/" else sys.argv[1]

        self.coordinates_source_path = sys.argv[2]
        self.border = int(sys.argv[3])  # search gcp in (1-border)% of the image, remove border% border around the image

    def run(self):

        start = time.time()
        self.read_gcp_file()

        # upload all filenames
        for filename in glob.glob(self.images_source_path + '*'):
            self.image_list.append(filename)

        total_images = len(self.image_list)

        if total_images == 0:
            sys.exit("Images directory is empty.")

        stats = Statistics(total_images)

        with exiftool.ExifTool() as et:
            metadata = et.get_tags_batch(self.keywords, self.image_list)

        for i in range(0, total_images):

            if len(metadata[i]) != 12 or self.check_metainfo(
                    metadata[i]) is False:  # Its required 12 specific parameters to process the gcp location
                self.missing = True

        if not self.missing:

            print("\nGPS information found!\n")
            print("Proceding to search for images that probably have a GCP in it.\n")

            for i in range(0, total_images):

                current_image = self.make_image(metadata[i])  # create a new instance of Class Image

                print("Current image:", current_image.get_filename())

                self.SENSOR_WIDTH = self.get_drone_info(Image_.get_drone_model())

                if self.SENSOR_WIDTH == 0:
                    sys.exit("Sensor Width is 0")

                distance = current_image.get_altitude() / math.tan(current_image.get_pitch_angle() * math.pi / 180)
                distance = distance / 1000  # m to km
                self.show_info(current_image)
                origin = geopy.Point(current_image.get_latitude(), current_image.longitude)
                destination = geopy.distance.GeodesicDistance(kilometers=distance).destination(origin, current_image.
                                                                                               get_horizontal_angle())
                lat2, lon2 = destination.latitude, destination.longitude

                print(self.is_gcp_nearby((lat2, lon2), current_image, stats))
                print()
                stats.save_statistic(1, "meta")

        else:
            print("\nGPS information not found!\n")
            print("Proceding to search for GCPs in all images.\n")

        self.write_gcp_file_header()
        self.aruco_detect(stats)
        end = time.time()
        print("Elapsed time", round(end - start, 1), "s")

    def aruco_detect(self, stats):
        marker_found = 0

        # if there is metadata in the images, we search for gcps in the ones selected
        number_of_images = len(self.images_with_gcp)
        # if there isn't, we process every uploaded image
        if number_of_images == 0:
            number_of_images = len(self.image_list)
            stats.update_aruco(number_of_images)
            with exiftool.ExifTool() as met:
                meta = met.get_tags_batch(self.keywords, self.image_list)
        else:
            stats.update_aruco(number_of_images)
            with exiftool.ExifTool() as met:
                meta = met.get_tags_batch(self.keywords, self.images_with_gcp)

        for k in range(0, number_of_images):

            vec = []
            image_meta = meta[k]
            image_filename = image_meta["SourceFile"]
            state = False

            frame = cv2.imread(image_filename)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Dictionary with 16 bits markers and ids from 0 to 49
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()

            parameters.cornerRefinementMaxIterations = 20
            parameters.cornerRefinementMethod = 0
            parameters.polygonalApproxAccuracyRate = 0.1
            parameters.cornerRefinementWinSize = 5
            parameters.cornerRefinementMinAccuracy = 0.08
            parameters.perspectiveRemovePixelPerCell = 4
            parameters.maxErroneousBitsInBorderRate = 0.04
            parameters.adaptiveThreshWinSizeStep = 2
            parameters.adaptiveThreshWinSizeMax = 21
            parameters.perspectiveRemoveIgnoredMarginPerCell = 0.4
            parameters.minMarkerPerimeterRate = 0.008

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters)

            if ids is not None:
                for j in range(len(ids)):
                    c = corners[j][0]
                    center_point = [c[:, 0].mean()], [c[:, 1].mean()]
                    vec.append(center_point)

                if ([-1], [-1]) not in vec:
                    state = self.addLine(vec, image_filename, ids)

                if state:
                    print("Marker found!", self.image_list[k])
                    marker_found = marker_found + 1
                    stats.save_statistic(1, "gcp_found")
                    if self.save_images == 1:
                        self.save_images_to_folder(image_filename)
            else:
                print("Marker not found in image", self.image_list[k])

            stats.save_statistic(1, "aruco")

        print("\nFound", marker_found, "markers out of", stats.get_total_images(), "images uploaded.")

    def addLine(self, pixels, filename_, gcp_ids):
        sucess = False
        im = os.path.split(filename_)
        img_name, img_extension = im[-1].split('.')
        s = 0
        for m in gcp_ids:
            n = m[0]
            try:
                gcp = self.get_gcp_info(n)
                # longitude, latitude, altitude, imagem_pixel_X, image_pixel_Y, image_name, gcp id
                line = str(gcp.get_lat()) + " " + str(gcp.get_long()) + " " + str(gcp.get_alt()) + " " + str(
                    pixels[s][0][0]) + \
                       " " + str(pixels[s][1][0]) + " " + img_name + " " + str(n) + "\n"

                gcp_file_location = self.save_gcp_path + "/gcp_list.txt"
                f = open(gcp_file_location, 'a')
                f.write(line)
                f.close()

                s += 1
                sucess = True
            except KeyError:
                print("Incorrect reading. Do not print." + " False identification with ID ->", n, "in " + img_name)
            return sucess

    def check_metainfo(self, metainfo):
        correct_meta = True
        for word in self.keywords:
            if word not in metainfo:
                correct_meta = False

        return correct_meta

    def save_images_to_folder(self, image_path):
        # guardar imagem numa pasta a parte
        img_ = os.path.split(image_path)
        img_name, img_extension = img_[-1].split('.')
        os.makedirs(os.path.dirname(self.save_images_path), exist_ok=True)
        shutil.copy(image_path, self.save_images_path + img_name + "." + img_extension)

    def write_gcp_file_header(self):
        gcp_file_location = self.save_gcp_path + "/gcp_list.txt"
        f = open(gcp_file_location, 'w+')
        f.write(self.lista_de_GCP_fixos[0].get_format_())
        f.close()

    def read_gcp_file(self):

        try:
            f = open(self.coordinates_source_path, 'r')
        except OSError:
            print("Could not open/read file:", self.coordinates_source_path)
            sys.exit()

        header = f.readline()
        for ln in f:
            line = ln.split()
            if len(line) > 0:  # read in format ID Lat Long Alt
                gcp = GroundControlPoint(int(line[0]), float(line[1]), float(line[2]),
                                         float(line[3]), header)
                self.lista_de_GCP_fixos[gcp.get_id()] = gcp

    @staticmethod
    def get_border_scale(b):
        if type(b) not in [int, float]:
            raise TypeError("The border percentage has to be an int or float")
        if b < 0:
            raise TypeError("The border percentage has to be positive")
        if b >= 100:
            raise TypeError("The border percentage cannot be equal or greater than 100%")
        elif b == 0:
            return 1
        img_without_border = abs((b / 100) - 1)
        scale_raw = math.sqrt(img_without_border)
        scale = (abs(scale_raw - 1)) / 2
        return scale

    def get_distance_to_corners(self, image):
        border_estimated = self.get_border_scale(self.border)

        ground_sample_distance = (image.get_altitude() * self.SENSOR_WIDTH) / (
                image.get_focal_length() * image.get_image_width())  # m/pixel

        # margem por lado (largura no [0] e altura no [1])
        bor = (image.get_image_width() * border_estimated), image.get_image_height() * border_estimated

        center_point = image.get_image_width() / 2, image.get_image_height() / 2

        dist = math.sqrt((center_point[0] - bor[0]) ** 2 + (center_point[1] - bor[1]) ** 2)  # distance in pixels

        final_distance = dist * ground_sample_distance  # real distance in meters

        return final_distance

    @staticmethod
    def get_drone_info(model):
        f = open('drones_DB.json')
        data = json.load(f)

        return data[model]

    def show_info(self, image):

        gsdW = (image.get_altitude() * self.SENSOR_WIDTH) / (
                image.get_focal_length() * image.get_image_width())  # m/pixel

        print("Altitude:", image.get_altitude(), "m")
        print("Sensor width:", self.SENSOR_WIDTH, "m")
        print("Focal lenght:", image.get_focal_length(), "m")
        print("Image width:", image.get_image_width(), "px")
        print("Ground Sample Distance:", round(gsdW * 100, 5), "cm/pixel")

    @staticmethod
    def make_image(meta):
        pitch_angle = abs(meta["MakerNotes:CameraPitch"])  # pitch_angle has to be positive
        image_width = meta["File:ImageWidth"]
        image_height = meta["File:ImageHeight"]
        focal_length = meta["EXIF:FocalLength"] / 1000  # mm to m
        horizontal_angle = float(meta["MakerNotes:Yaw"])  # O yaw e calculado em sentido clockwise
        altitude = float(meta["XMP:RelativeAltitude"][1:-1])  # raw altitude value
        filename = meta["SourceFile"]
        model = meta["EXIF:Model"]
        lati = meta['EXIF:GPSLatitude']
        lon = meta['EXIF:GPSLongitude']
        latRef = meta['EXIF:GPSLatitudeRef']
        longRef = meta['EXIF:GPSLongitudeRef']

        # avoid division by 0 -> if pitch_angle == 0, drone is looking to horizon
        if pitch_angle == 0:
            pitch_angle = 0.000001

        if latRef == "S":
            lati = -lati
        elif longRef == "W":
            lon = -lon

        return Image_(pitch_angle, image_width, image_height, focal_length, horizontal_angle, altitude, filename,
                      model,
                      lati, lon)

    def get_gcp_info(self, id__):
        return self.lista_de_GCP_fixos[id__]

    # receive the gps coordinates of the focal point in the image and the image itself
    def is_gcp_nearby(self, centerCoord, img, stats):
        top_right, bottom_right, bottom_left, top_left = self.get_corner_coordinates(img, centerCoord[0],
                                                                                     centerCoord[1])

        image_path = img.get_filename()
        find = False
        # assuming list GCP already in DD format
        for gcp in self.lista_de_GCP_fixos:

            p = LatLon(self.lista_de_GCP_fixos[gcp].get_lat(), self.lista_de_GCP_fixos[gcp].get_long())
            b = LatLon(top_right.latitude, top_right.longitude), LatLon(bottom_right.latitude, bottom_right.longitude), \
                LatLon(bottom_left.latitude, bottom_left.longitude), LatLon(top_left.latitude, top_left.longitude)

            if p.isenclosedBy(b):
                find = True
                stats.save_statistic(1, "contains_gcp")
                if image_path not in self.images_with_gcp:
                    self.images_with_gcp.append(image_path)

        if find:
            return self.found
        else:
            return self.not_found

    def get_corner_coordinates(self, img, centerLat, centerLong):

        # angle between the top and the right cornor of the image (it will be 45 degrees if the image its a square)
        angle = math.atan((img.get_image_width() / 2) / (img.get_image_height() / 2)) * (180.0 / math.pi)

        NE = angle  # starting from North which is 0
        SE = 180 - angle
        SW = 180 + angle
        NW = 360 - angle

        dist = self.get_distance_to_corners(img)

        dist = dist / 1000  # in km

        center_point_coord = geopy.Point(centerLat, centerLong)

        top_right = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                                 img.get_horizontal_angle() + NE)
        bottom_right = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                                    img.get_horizontal_angle() + SE)
        bottom_left = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                                   img.get_horizontal_angle() + SW)
        top_left = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                                img.get_horizontal_angle() + NW)

        return top_right, bottom_right, bottom_left, top_left


if __name__ == '__main__':
    finder = GCPFinder()
    finder.run()
