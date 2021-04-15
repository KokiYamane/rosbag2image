import shutil
import os
import sys
import glob
import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm

if os.path.exists('data'):
    shutil.rmtree('data')

os.mkdir('data')

bagfilesfolder = sys.argv[1]
bagfilenames = glob.glob('{}/*.bag'.format(bagfilesfolder))

for bagfilename in bagfilenames:
    bagfilename_splited = os.path.splitext(os.path.basename(bagfilename))[0]
    folder = 'data/' + bagfilename_splited

    os.mkdir(folder)
    os.mkdir(folder + '/color')
    os.mkdir(folder + '/depth')
    # os.mkdir(folder + 'pcd')

    image_color_list = []
    image_depth_list = []
    timestamp_list = []
    try:
        print(bagfilename_splited)
        for topic, msg, t in rosbag.Bag(bagfilename).read_messages():
            # if topic == '/device_0/sensor_1/Color_0/image/data':
            if topic == '/eye_to_hand_camera/color/image_raw':
            # if topic == '/eye_on_hand_camera/color/image_raw':
                image_color = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
                image_color_list.append(image_color)
                timestamp_list.append(t)

            # if topic == '/device_0/sensor_0/Depth_0/image/data':
            if topic == '/eye_to_hand_camera/aligned_depth_to_color/image_raw':
            # if topic == '/eye_on_hand_camera/aligned_depth_to_color/image_raw':
                image_depth = CvBridge().imgmsg_to_cv2(msg, 'passthrough')
                image_depth_list.append(image_depth)


        for timestamp, image_color, image_depth in tqdm(zip(
                timestamp_list, image_color_list, image_depth_list), total=len(timestamp_list)):
            t = timestamp.to_nsec()
            # image_color = cv2.resize(image_color, (image_depth.shape[1], image_depth.shape[0]))
            cv2.imwrite(folder + '/color/color{:012}.png'.format(t), image_color)
            cv2.imwrite(folder + '/depth/depth{:012}.png'.format(t), image_depth)

            # color_raw = o3d.io.read_image(folder + '/color/color{:012}.png'.format(t))
            # depth_raw = o3d.io.read_image(folder + '/depth/depth{:012}.png'.format(t))
            # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            #     color_raw, depth_raw, convert_rgb_to_intensity=False)
            # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            #     rgbd_image,
            #     o3d.camera.PinholeCameraIntrinsic(
            #         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
            # o3d.io.write_point_cloud(folder + '/pcd/pcd{:012}.pcd'.format(t), pcd)
    except:
        print(bagfilename_splited + ' can\'t convert')
