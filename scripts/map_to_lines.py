#!/usr/bin/env python2

'''
This program converts ROS map server data to line segments. This is because
the performance of Box2D (mainly Raycasts for lasers) is dependent on the number
of line segments. Maps generated from laser data tends to be noisy and contains
a large number tiny line segments when extracted by flatland, up to hundreds of
thousands for a large map. This program uses OpenCV image processing to extract
line segments while sacrificing some accuracy.

run with --help to see more options
'''

import numpy as np
import argparse
import cv2
import yaml
import os

def main():
    # get all the arguments
    arg_parser = argparse.ArgumentParser(description="Generate line segment data file from ROS map server data")
    arg_parser.add_argument("map_yaml_path", help="path to the map server yaml file")
    arg_parser.add_argument("output_path", 
        help="path of the directory to output the line segment map files (.yaml and .dat files)")
    arg_parser.add_argument("-f", "--filename", dest="filename", default="map_lines", 
        help="name of the line segment map files (extensions .yaml and .dat will be added automatically)")
    arg_parser.add_argument("-i", "--output-image", dest="output_image", action='store_true', 
        help="Output image with line segments drawn on top of the map image")
    arg_parser.add_argument("-d", "--definition", dest="definition", choices=["low", "med", "high"], default="low", 
        help="how detailed you want the linesegments to represent the map")    
    args = arg_parser.parse_args()

    map_yaml_path = os.path.abspath(args.map_yaml_path)
    output_path = os.path.abspath(args.output_path)
    filename = args.filename
    output_image = args.output_image
    definition = args.definition
    
    # extract data from map server format
    node = yaml.load(open(map_yaml_path, "r"))
    image_path = os.path.join(os.path.dirname(map_yaml_path), node["image"])
    scale = node["resolution"]
    origin = node["origin"]
    occupied_thresh = node["occupied_thresh"]

    print("")
    print("From input map yaml file: ")
    print("          image: %s" % image_path)
    print("          scale: %f" % scale)
    print("         origin: [%f, %f, %f]" % (origin[0], origin[1], origin[2]))
    print("occupied_thresh: %f" % occupied_thresh)

    output_yaml_path = os.path.join(output_path, filename + ".yaml")
    output_lines_path = os.path.join(output_path, filename + ".dat")
    output_image_path = os.path.join(output_path, filename + ".png")

    print("")
    print("Output files are:")
    print("  map: %s" % output_yaml_path)
    print("lines: %s" % output_lines_path)

    if output_image:
        print("Image: %s" % output_image_path)

    print("")
    print("Processing...")

    # threshold the image
    raw_img = cv2.imread(image_path, 0)
    img = cv2.inRange(raw_img, occupied_thresh * 255, 255.0)
    
    # cv2.createLineSegmentDetector([_refine[, _scale[, _sigma_scale[, _quant[, _ang_th[, _log_eps[, _density_th[, _n_bins]]]]]]]])
    # Read the paper from http://www.ipol.im/pub/art/2012/gjmr-lsd/ to get an understanding of the parameters
    # play around with the parameters to obtain the desired results
    # _refine: does not have much of a effect, using standard should be fine
    # _scale (default = 0.8)
    # _sigma_scale (default = 0.6)
    # _quant (default = 2)
    # _ang_th (default = 22.5)
    # _log_eps (default = 1) 
    # _density_th (default = 0.7)
    # _n_bins (default = ?)
    
    if definition == "low":
        lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD, 
            1.05, 1, 1, 22.5, 1, 0.8)
    elif definition == "med":
        lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD, 
            1.5, 1, 1, 22.5, 1, 0.8)
    elif definition == "high":
        lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD, 
            2, 1, 1, 22.5, 1, 0.8)
    
    lines = lsd.detect(img)[0]
    
    print("%d line segments generated\n" % len(lines))
    
    # generate the line segment file
    output_lines_file = open(output_lines_path, "w")
    rows = np.size(img, 0)
    for line in lines:
        line = line[0]
        output_lines_file.write("%25.15f %25.15f %25.15f %25.15f\n" % 
            (line[0], rows - line[1], line[2], rows - line[3]))
    
    output_lines_file.close()

    # generate the map yaml file
    output_yaml = {}
    output_yaml["type"] = "line_segments"
    output_yaml["data"] = os.path.basename(output_lines_path)
    output_yaml["scale"] = scale
    output_yaml["origin"] = origin

    yaml.dump(output_yaml, open(output_yaml_path, "w"))
    
    #Draw detected lines in the image
    if output_image:
        drawn_img = lsd.drawSegments(img, lines)
        cv2.imwrite(output_image_path, drawn_img)

if __name__ == "__main__":
    main()