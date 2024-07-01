import sys
import numpy as np
import pyzed.sl as sl
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

help_string = "[d] Save Depth, [n] Change Depth format, [q] Quit"
prefix_depth = "Depth_"
path = "./"

count_save = 0
mode_depth = 0
depth_format_ext = ".png"

def depth_format_name():
    global mode_depth
    if mode_depth > 2:
        mode_depth = 0
    switcher = {
        0: ".png",
        1: ".pfm",
        2: ".pgm",
    }
    return switcher.get(mode_depth, "nothing")

def save_depth(zed, filename):
    print("Saving Depth Map...")
    tmp = sl.Mat()
    zed.retrieve_measure(tmp, sl.MEASURE.DEPTH)
    saved = (tmp.write(filename + depth_format_ext) == sl.ERROR_CODE.SUCCESS)
    if saved:
        print("Done")
    else:
        print("Failed... Please check that you have permissions to write on disk")

def process_key_event(zed, key):
    global mode_depth
    global count_save
    global depth_format_ext

    if key == 100 or key == 68:
        save_depth(zed, path + prefix_depth + str(count_save))
        count_save += 1
    elif key == 110 or key == 78:
        mode_depth += 1
        depth_format_ext = depth_format_name()
        print("Depth format: ", depth_format_ext)
    elif key == 113 or key == 81:
        return True
    return False

def print_help():
    print(" Press 'd' to save Depth image")
    print(" Press 'n' to switch Depth format")
    print(" Press 'q' to Quit")

def main():
    # Initialize ROS node
    rospy.init_node('zed_depth_publisher', anonymous=True)
    depth_pub = rospy.Publisher('zed_depth', Image, queue_size=10)
    bridge = CvBridge()

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    if len(sys.argv) >= 2:
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

    # Display help in console
    print_help()

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_configuration.resolution
    image_size.width = image_size.width // 2
    image_size.height = image_size.height // 2

    # Declare your sl.Mat matrices
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    key = ' '
    while key != 113 and not rospy.is_shutdown():
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            # Retrieve the depth image in the half-resolution
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            depth_image_ocv = depth_image_zed.get_data()

            # Print the depth image numpy array
            print("Depth Image Data:")
            print(depth_image_ocv)

            # Display the depth image
            cv2.imshow("Depth", depth_image_ocv)

            # Publish the depth image to ROS topic
            depth_msg = bridge.cv2_to_imgmsg(depth_image_ocv, encoding="passthrough")
            depth_pub.publish(depth_msg)

            key = cv2.waitKey(10)

            if process_key_event(zed, key):
                break

    cv2.destroyAllWindows()
    zed.close()

    print("\nFINISH")

if __name__ == "__main__":
    main()
