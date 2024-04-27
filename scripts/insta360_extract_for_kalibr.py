import cv2
import os
from subprocess import Popen
from os.path import join as pjoin
from argparse import ArgumentParser
from telemetry_converter import TelemetryConverter
from utils import time_to_s_nsec
import shutil

def extract_frames(input_video, img_timestamps, output_image_path, downsample_fac=2.0, skip_frames=0):
    if not os.path.exists(output_image_path):
        os.makedirs(output_image_path)
    cap = cv2.VideoCapture(input_video)
    count = 0
    empty_frame = 0
    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    assert len(img_timestamps) == num_frames, f"Number of frames in video ({num_frames}) does not match number of timestamps ({len(img_timestamps)})"
    while (cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if count % (skip_frames + 1) == 0:
            if ret:
                timestamp_ns = img_timestamps[count]
                timestamp_s, timestamp_ns = time_to_s_nsec(timestamp_ns / 1e9)
                if count % 100 == 0:
                    print('Wrote frame {}/{}: '.format(count,num_frames))
                frame = cv2.resize(frame, (0,0), fx=1/downsample_fac, fy=1/downsample_fac)
                cv2.imwrite(os.path.join(output_image_path, "{0}{1:09d}.png".format(timestamp_s, timestamp_ns)), frame) 
            else:
                if empty_frame > 200:
                    break
                empty_frame += 1
        count += 1

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = ArgumentParser()
    parser.add_argument("-i", '--input_video', 
                        help="Path to the mp4 video",
                        required=True)
    parser.add_argument('--downsample_fac', default=2, type=float)
    parser.add_argument('--skip_frames', default=0, type=int)   
    args = parser.parse_args()

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file, "..")
    telemetry_conv = TelemetryConverter()

    # create output folder
    output_path = args.input_video.rsplit('/', 1)[0] + "/bag_input"
    if os.path.exists(output_path):
        shutil.rmtree(output_path)
    os.makedirs(output_path)

    # extract imu data
    cam_raw_telemetry = args.input_video.replace('.mp4', '_raw_telemetry.txt')
    cam_processed_telemetry = pjoin(output_path,"imu0.csv")
    with open(cam_raw_telemetry, 'w') as f:
        cam_extraction = Popen([
            f"{path_to_src}/Thirdparty/exiftool/exiftool", 
            "-m", "-ee", args.input_video
        ], stdout=f)
        cam_extraction.wait()
    telemetry_conv.convert_insta360_telemetry_to_kalibr(args.input_video, cam_raw_telemetry, cam_processed_telemetry)

    # extract frames
    img_timestamps = telemetry_conv.telemetry_importer.telemetry["img_timestamps_ns"]
    extract_frames(args.input_video, img_timestamps, os.path.join(output_path,'cam0'), skip_frames=args.skip_frames)

if __name__=="__main__":
    main()