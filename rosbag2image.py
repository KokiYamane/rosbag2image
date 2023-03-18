import shutil
import os
import glob
import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm


def main(args):

    if os.path.exists(args.output):
        shutil.rmtree(args.output)
    os.mkdir(args.output)

    bagfilenames = glob.glob(f'{args.bagfilesfolder}/*.bag')
    for bagfilename in bagfilenames:
        bagfilename_splited = os.path.splitext(
            os.path.basename(bagfilename))[0]
        folder = os.path.join(args.output, bagfilename_splited)

        os.mkdir(folder)
        os.mkdir(folder + '/color')
        os.mkdir(folder + '/depth')

        image_color_list = []
        image_depth_list = []
        timestamp_list = []
        try:
            print(bagfilename_splited)
            for topic, msg, t in tqdm(rosbag.Bag(bagfilename).read_messages()):
                if topic == args.topic_name_color:
                    image_color = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
                    image_color_list.append(image_color)
                    timestamp_list.append(t)

                if topic == args.topic_name_depth:
                    image_depth = CvBridge().imgmsg_to_cv2(msg, 'passthrough')
                    image_depth_list.append(image_depth)

            for timestamp, image_color, image_depth in tqdm(zip(
                timestamp_list,
                image_color_list,
                image_depth_list
            ), total=len(timestamp_list)):
                t = timestamp.to_nsec()
                cv2.imwrite(f'{folder}/color/color{t:012}.jpg', image_color)
                cv2.imwrite(f'{folder}/depth/depth{t:012}.jpg', image_depth)

        except:
            print(bagfilename_splited + ' can\'t convert')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfilesfolder')
    parser.add_argument('--topic_name_color',
                        default='/device_0/sensor_1/Color_0/image/data')
    parser.add_argument('--topic_name_depth',
                        default='/device_0/sensor_0/Depth_0/image/data')
    parser.add_argument('--output', default='./data')
    args = parser.parse_args()
    main(args)
