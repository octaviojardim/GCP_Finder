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
from PIL import Image, ImageDraw
from cv2 import aruco
from pygeodesy.sphericalNvector import LatLon
from Image_ import Image_
from GroundControlPoint import GroundControlPoint
from Statistics import Statistics

import matplotlib.pyplot as plt


class GCPFinder:
    found = "Ponto de Controle encontrado"
    not_found = "Ponto de Controle NÃO encontrado"
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
            for i in range(0, total_images):
                print("total_images:", total_images)
                print("metadata:", len(metadata))

                current_image = self.make_image(metadata[i])  # create a new instance of Class Image

                print("current image:", current_image)
                print("Initial coordinates:", current_image.get_latitude(), current_image.get_longitude())

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
                print("Final coordinates:", lat2, lon2)

                # INFO:
                print("Distancia projetada:", round(distance * 1000, 2), "m")
                print(self.is_gcp_nearby((lat2, lon2), current_image, stats))
                print()
                stats.save_statistic(1, "meta")

        self.write_gcp_file_header()
        self.aruco_detect(stats)
        end = time.time()
        print("Elapsed time", round(end - start, 1), "s")

    @staticmethod
    def get_center_point(corners):
        topLeft = corners[0]
        topRight = corners[1]
        bottomRight = corners[2]
        bottomLeft = corners[3]

        line1 = (topLeft, bottomRight)
        line2 = (bottomLeft, topRight)

        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)

        if div == 0:
            print("Lines do not intersect.")
            return [-1], [-1]

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        plt.plot(x, y, ".", color='Red')

        return [x], [y]

    def aruco_detect(self, stats):
        marker_found = 0

        # if there is metadata in the images, we search for gcp in the ones selected
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

        print("number_of_images", number_of_images)
        for k in range(0, number_of_images):

            vec = []
            image_meta = meta[k]
            image_filename = image_meta["SourceFile"]

            frame = cv2.imread(image_filename)
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # kernel = np.ones((3, 3), np.uint8)
            # low_rgb = np.array([100, 100, 100])
            # high_rgb = np.array([255, 255, 255])
            # black_white = cv2.inRange(frame, low_rgb, high_rgb)
            # closing = cv2.morphologyEx(black_white, cv2.MORPH_CLOSE, kernel)
            # opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)

            '''Experimentar LUT e depois gray com default parameters
            LUT_IN = [0, 158, 216, 255]
            LUT_OUT = [0, 22, 80, 176]
            self.lut = np.interp(np.arange(0, 256), self.LUT_IN,
                             self.LUT_OUT).astype(np.uint8)
            tmp = cv2.LUT(frame, self.lut)
            gray = cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY)
            
            '''

            # Dictionary with 16 bits markers and ids from 0 to 49
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()

            parameters.cornerRefinementMaxIterations = 20#80
            parameters.cornerRefinementMethod = 0#1
            parameters.polygonalApproxAccuracyRate = 0.1#0.05
            parameters.cornerRefinementWinSize = 5#20
            parameters.cornerRefinementMinAccuracy =0.08 #0.05
            parameters.perspectiveRemovePixelPerCell = 4#8
            parameters.maxErroneousBitsInBorderRate = 0.04#0.02
            parameters.adaptiveThreshWinSizeStep = 2  # alterei
            parameters.adaptiveThreshWinSizeMax = 21#23
            parameters.perspectiveRemoveIgnoredMarginPerCell = 0.29#0.4  # alterei
            parameters.minMarkerPerimeterRate = 0.01#0.008  # alterei

            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            dummy = frame.copy()
            # for rejected in rejectedImgPoints:
            #    rejected = rejected.reshape((4, 2))
            #    cv2.line(dummy, tuple(rejected[0]), tuple(rejected[1]), (0, 0, 255), thickness=2)
            #    cv2.line(dummy, tuple(rejected[1]), tuple(rejected[2]), (0, 0, 255), thickness=2)
            #    cv2.line(dummy, tuple(rejected[2]), tuple(rejected[3]), (0, 0, 255), thickness=2)
            #    cv2.line(dummy, tuple(rejected[3]), tuple(rejected[0]), (0, 0, 255), thickness=2)

            # newwwwww
            # dummy2 = aruco.drawDetectedMarkers(dummy, rejectedImgPoints, ids)
            # frame_markers = aruco.drawDetectedMarkers(dummy, corners, ids)
            # plt.figure()
            # plt.imshow(frame_markers)

            if ids is not None:
                for j in range(len(ids)):
                    c = corners[j][0]
                    center_point = self.get_center_point(c)
                    # pixels = [c[:, 0].mean()], [c[:, 1].mean()]
                    # plt.imshow(dummy)
                    # plt.show()

                    vec.append(center_point)
                    state = False
                if ([-1], [-1]) not in vec:
                    state = self.addLine(vec, image_filename, ids)

                if state:
                    print("Marker found!", self.image_list[k], "id:", ids)
                    marker_found = marker_found + 1
                    stats.save_statistic(1, "gcp_found")
                    if self.save_images == 1:
                        self.save_images_to_folder(image_filename)
            else:
                print("Marker not found in image", self.image_list[k])

            stats.save_statistic(1, "aruco")

        print("Found", marker_found, "of", stats.get_total_images(), "markers")

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
                print("Incorrect reading. Do not print." + " ID->", n, "on " + img_name)
            return sucess

    def check_metainfo(self, metainfo):
        correct_meta = True
        for word in self.keywords:
            if word not in metainfo:
                correct_meta = False

        return correct_meta

    def save_images_to_folder(self, image_path):
        # guardar imagem numa pasta à parte
        img_ = os.path.split(image_path)
        img_name, img_extension = img_[-1].split('.')
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
            if len(line) > 0:
                gcp = GroundControlPoint(int(line[0]), float(line[2]), float(line[1]),
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

        print("border_estimated:", border_estimated)

        ground_sample_distance = (image.get_altitude() * self.SENSOR_WIDTH) / (
                image.get_focal_length() * image.get_image_width())  # m/pixel

        # margem por lado (largura no [0] e altura no [1])
        bor = (image.get_image_width() * border_estimated), image.get_image_height() * border_estimated

        center_point = image.get_image_width() / 2, image.get_image_height() / 2

        dist = math.sqrt((center_point[0] - bor[0]) ** 2 + (center_point[1] - bor[1]) ** 2)  # distance in pixels

        final_distance = dist * ground_sample_distance  # real distance in meters

        print("final_distance:", final_distance)

        # DRAW TOP LEFT AND RIGHT CORNERS IN IMAGE
        # shape = [(int(p[0]) - 2, int(p[1]) - 2), (int(p[0]) + 2, int(p[1]) + 2)]
        # shape2 = [image.get_image_width() - (int(p[0]) - 2), int(p[1]) - 2, image.get_image_width() - (int(p[0]) + 2), int(p[1]) + 2]
        # img = Image.open(self.image_list[0])
        # img1 = ImageDraw.Draw(img)
        # img1.rectangle(shape, fill="#ffff33", outline="red")
        # img1.rectangle(shape2, fill="#ffff33", outline="red")
        # img.show()

        #retangulo branco com a margem
        '''img = Image.open(self.image_list[0])
        img1 = ImageDraw.Draw(img, "RGBA")

        img1.rectangle([(0, 0), (image.get_image_width(), bor[1])], (255, 255, 255, 100))
        img1.rectangle([(0, bor[1]+1), (bor[0], image.get_image_height())], (255, 255, 255, 100))
        img1.rectangle(
            [(bor[0]+1, image.get_image_height() - bor[1]), (image.get_image_width(), image.get_image_height())],
            (255, 255, 255, 100))
        img1.rectangle(
            [(image.get_image_width() - bor[0], image.get_image_height() - bor[1]-1), (image.get_image_width(), bor[1]+1)],
            (255, 255, 255, 100))
        del img1
        #img.show()
        img.save('/Users/octaviojardim/Desktop/margem.png', 'PNG')'''

        return final_distance

    @staticmethod
    def get_drone_info(model):
        f = open('drones_DB.json')
        data = json.load(f)

        return data[model]

    def show_info(self, image):
        gsdW = (image.get_altitude() * self.SENSOR_WIDTH) / (
                image.get_focal_length() * image.get_image_width())  # m/pixel

        print("Altitude: ", image.get_altitude(), "m")
        print("Sensor width: ", self.SENSOR_WIDTH, "m")
        print("Focal lenght: ", image.get_focal_length(), "m")
        print("Image width: ", image.get_image_width(), "px")
        print("Ground Sample Distance: ", gsdW * 100, "cm/pixel")

    @staticmethod
    def make_image(meta):
        pitch_angle = abs(meta["MakerNotes:CameraPitch"])  # pitch_angle has to be positive
        image_width = meta["File:ImageWidth"]
        image_height = meta["File:ImageHeight"]
        focal_length = meta["EXIF:FocalLength"] / 1000  # mm to m
        horizontal_angle = float(meta["MakerNotes:Yaw"])  # O yaw é calculado em sentido clockwise
        altitude = float(meta["XMP:RelativeAltitude"][1:-1])  # raw altitude value
        filename = meta["SourceFile"]
        model = meta["EXIF:Model"]
        lati = meta['EXIF:GPSLatitude']
        lon = meta['EXIF:GPSLongitude']
        latRef = meta['EXIF:GPSLatitudeRef']
        longRef = meta['EXIF:GPSLongitudeRef']

        # avoid division by 0 -> if pitch_angle == 0, drone is looking to
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

    # recebe as coordenadas gps do ponto focal da imagem e a imagem em questão
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
                print("gcp found inside image")
                stats.save_statistic(1, "contains_gcp")
                if image_path not in self.images_with_gcp:
                    self.images_with_gcp.append(image_path)

        if find:
            return self.found
        else:
            return self.not_found

    def get_corner_coordinates(self, img, centerLat, centerLong):

        # angle between the top and the right cornor of the image (it will be 45º if the image its a square)
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

        print("top_right", top_right.latitude, top_right.longitude)
        print("bottom_right", bottom_right.latitude, bottom_right.longitude)
        print("bottom_left", bottom_left.latitude, bottom_left.longitude)
        print("top_left", top_left.latitude, top_left.longitude)

        return top_right, bottom_right, bottom_left, top_left


if __name__ == '__main__':
    finder = GCPFinder()
    finder.run()
