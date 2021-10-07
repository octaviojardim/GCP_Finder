import os
import cv2
import sys
import math
import glob
import shutil
import exiftool
import geopy.distance
from cv2 import aruco
from PIL import Image, ImageDraw
from pygeodesy.sphericalNvector import LatLon

# CONSTANTES
lista_de_GCP_fixos = {
    0: (41.399764282, -6.979935297, 745),
    1: (41.399996133, -6.979810771, 756),
    13: (89.393838443, -6.456447899, 745)}

found = "Ponto de Controle encontrado"
not_found = "Ponto de Controle NÃO encontrado"
source_path = "/Users/octaviojardim/Desktop/source" + "/"
save_path = "/Users/octaviojardim/Desktop/images_selected" + "/"
keywords = ["EXIF:Model", "MakerNotes:Yaw", "MakerNotes:CameraPitch", "XMP:RelativeAltitude", "File:ImageWidth",
            "File:ImageHeight", "EXIF:FocalLength", "EXIF:GPSLatitude", "EXIF:GPSLongitude", "EXIF:GPSLatitudeRef",
            "EXIF:GPSLongitudeRef", "XMP:GimbalPitchDegree"]
output_lines = []
images_with_gcp = ["/Users/octaviojardim/Desktop/imagem_teste.png"]
image_list = []
img_with_gcp = 0
BORDER = 20  # search gcp in (1-BORDER)% of the image, remove BORDER% border around the image
save_images = True


def get_border_scale(border):
    img_without_border = abs((border / 100) - 1)
    scale_raw = math.sqrt(img_without_border)
    scale = (abs(scale_raw - 1)) / 2
    return scale


def get_distance_to_corners():
    global BORDER
    border_estimated = get_border_scale(BORDER)

    ground_sample_distance = (altitude * SENSOR_WIDTH) / (focal_length * image_width)  # m/pixel

    p = (image_width * border_estimated), image_height * border_estimated
    center_point = image_width / 2, image_height / 2

    dist = math.sqrt((center_point[0] - p[0]) ** 2 + (center_point[1] - p[1]) ** 2)  # distance in pixels

    final_distance = dist * ground_sample_distance  # real distance in meters

    # DRAW TOP LEFT AND RIGHT CORNERS IN IMAGE
    shape = [(int(p[0]) - 2, int(p[1]) - 2), (int(p[0]) + 2, int(p[1]) + 2)]
    shape2 = [image_width - (int(p[0]) - 2), int(p[1]) - 2, image_width - (int(p[0]) + 2), int(p[1]) + 2]
    img = Image.open(image_list[0])
    img1 = ImageDraw.Draw(img)
    img1.rectangle(shape, fill="#ffff33", outline="red")
    img1.rectangle(shape2, fill="#ffff33", outline="red")
    img.show()

    return final_distance


def get_corner_coodinates(centerLat, centerLong):
    angle = math.atan((image_width / 2) / (image_height / 2)) * (180.0 / math.pi)

    NE = angle  # starting from North which is 0
    SE = 180 - angle
    SW = 180 + angle
    NW = 360 - angle

    dist = get_distance_to_corners()

    dist = dist / 1000  # in km

    center_point_coord = geopy.Point(centerLat, centerLong)

    top_right = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                             horizontal_angle + NE)
    bottom_right = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                                horizontal_angle + SE)
    bottom_left = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                               horizontal_angle + SW)
    top_left = geopy.distance.GeodesicDistance(kilometers=dist).destination(center_point_coord,
                                                                            horizontal_angle + NW)

    print("top_right", top_right.latitude, top_right.longitude)
    print("bottom_right", bottom_right.latitude, bottom_right.longitude)
    print("bottom_left", bottom_left.latitude, bottom_left.longitude)
    print("top_left", top_left.latitude, top_left.longitude)

    return top_right, bottom_right, bottom_left, top_left


def get_drone_info(drone):
    model = drone["EXIF:Model"]

    if model == "FC6310":
        return 0.0132, 0.0088
    elif model == "FC7303":
        return 0.0063, 0.0047
    else:
        return 0, 0


def get_coordinates(geotags):
    lati = geotags['EXIF:GPSLatitude']
    lon = geotags['EXIF:GPSLongitude']
    latRef = geotags['EXIF:GPSLatitudeRef']
    longRef = geotags['EXIF:GPSLongitudeRef']

    if latRef == "S":
        lati = -lati
    elif longRef == "W":
        lon = -lon
    return lati, lon


def show_info():
    gsdW = (altitude * SENSOR_WIDTH) / (focal_length * image_width)  # m/pixel

    print("Altitude: ", altitude, "m")
    print("Sensor width: ", SENSOR_WIDTH, "m")
    print("Focal lenght: ", focal_length, "m")
    print("Image width: ", image_width, "px")
    print("Ground Sample Distance: ", gsdW * 100, "cm/pixel")


# recebe as coordenadas gps do ponto focal da imagem e a imagem em questão
def is_gcp_nearby(centerCoord, img):
    global img_with_gcp, lista_de_GCP_fixos

    top_right, bottom_right, bottom_left, top_left = get_corner_coodinates(centerCoord[0], centerCoord[1])

    image_path = img["SourceFile"]
    find = False
    # assuming list GCP already in DD format
    for gcp in lista_de_GCP_fixos:

        p = LatLon(lista_de_GCP_fixos[gcp][0], lista_de_GCP_fixos[gcp][1])
        b = LatLon(top_right.latitude, top_right.longitude), LatLon(bottom_right.latitude, bottom_right.longitude), \
            LatLon(bottom_left.latitude, bottom_left.longitude), LatLon(top_left.latitude, top_left.longitude)

        if p.isenclosedBy(b):
            find = True
            img_with_gcp = img_with_gcp + 1
            images_with_gcp.append(image_path)

            if save_images:
                # guardar imagem numa pasta à parte
                img_ = os.path.split(image_path)
                img_name, img_extension = img_[-1].split('.')
                shutil.copy(image_path, save_path + img_name + "." + img_extension)

    if find:
        return found
    else:
        return not_found


def get_image_data(meta):
    global keywords, pitch_angle, image_width, image_height, focal_length, horizontal_angle, altitude

    pitch_angle = abs(82)  # meta["MakerNotes:CameraPitch"]  has to be greater than 0 -------------------------------
    image_width = meta["File:ImageWidth"]
    image_height = meta["File:ImageHeight"]
    focal_length = meta["EXIF:FocalLength"] / 1000  # mm to m
    horizontal_angle = float(meta["MakerNotes:Yaw"])  # O yaw é calculado em sentido clockwise
    altitude = meta["XMP:RelativeAltitude"]

    return pitch_angle, image_width, image_height, focal_length, horizontal_angle, altitude


# upload all filenames
for filename in glob.glob(source_path + '*'):
    image_list.append(filename)

total_images = len(image_list)

with exiftool.ExifTool() as et:
    metadata = et.get_tags_batch(keywords, image_list)

for i in range(0, len(image_list)):

    current_image = metadata[i]
    print(current_image)
    SENSOR_WIDTH, SENSOR_HEIGHT = get_drone_info(current_image)

    if SENSOR_WIDTH == 0 or SENSOR_HEIGHT == 0:
        sys.exit("Sensor Width or Height is 0")

    lat, long = get_coordinates(current_image)
    print("Initial coordinates:", lat, long)

    pitch_angle, image_width, image_height, focal_length, horizontal_angle, altitude = get_image_data(current_image)

    altitude = float(altitude[1:-1])

    if pitch_angle > 0:
        distance = altitude / math.tan(pitch_angle * math.pi / 180)
        distance = distance / 1000  # in km
        show_info()
        origin = geopy.Point(lat, long)
        destination = geopy.distance.GeodesicDistance(kilometers=distance).destination(origin, horizontal_angle)
        lat2, lon2 = destination.latitude, destination.longitude
        print("Final coordinates:", lat2, lon2)

        # INFO:
        print("Distancia projetada:", round(distance * 1000, 2), "m")
        print(is_gcp_nearby((lat2, lon2), current_image))
        print()


def get_gcp_info(id__):
    return lista_de_GCP_fixos[id__]


def addLine(pixels, filename_, gcp_ids):
    im = os.path.split(filename_)
    img_name, img_extension = im[-1].split('.')
    s = 0
    for m in gcp_ids:
        n = m[0]
        gcp_lat, gcp_long, gcp_alt = get_gcp_info(n)
        # longitude, latitude, altitude, imagem_pixel_X, image_pixel_Y, image_name, gcp id
        line = str(gcp_long) + " " + str(gcp_lat) + " " + str(gcp_alt) + " " + str(pixels[s][0][0]) + \
               " " + str(pixels[s][1][0]) + " " + img_name + " " + str(n) + "\n"
        output_lines.append(line)
        s = s + 1


def aruco_detect():
    marker_found = 0
    vec = []

    with exiftool.ExifTool() as met:
        meta = met.get_tags_batch(keywords, images_with_gcp)

    number_of_images = len(images_with_gcp)
    print("Number of images with gcp:", number_of_images)
    for k in range(0, number_of_images):
        image_meta = meta[k]
        image_filename = image_meta["SourceFile"]

        frame = cv2.imread(image_filename)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Dictionary with 16 bits markers and ids from 0 to 49
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            print("Marker found!")
            marker_found = marker_found + 1
            for j in range(len(ids)):
                c = corners[j][0]
                pixels = [c[:, 0].mean()], [c[:, 1].mean()]
                vec.append(pixels)
            addLine(vec, image_filename, ids)
        else:
            print("Marker not found in image", image_list[k])
            print("Found", marker_found, "of", total_images, "markers")


def generate_gcp_file():
    header = "WGS84\n"

    if len(output_lines) < 1:
        sys.exit("Didn't find any markers.")
    # in unix systems
    gcp_file_location = (os.path.join(os.path.join(os.path.expanduser('~')), 'Desktop')) + "/gcp_list.txt"
    f = open(gcp_file_location, 'w+')
    f.write(header)
    f.writelines(output_lines)


print("Numero de imagens com um ponto de controlo: ", img_with_gcp, "/", total_images)

coords_1 = (32.651917, -16.941869)
coords_2 = (32.651919280258994, -16.941869478180877)

print("GUESS ERROR", round(geopy.distance.geodesic(coords_1, coords_2).meters, 2), "m")

aruco_detect()
generate_gcp_file()
