import os
from argparse import ArgumentParser
from subprocess import Popen
from os.path import join as pjoin
import glob
from telemetry_converter import TelemetryConverter


def main():
    parser = ArgumentParser("OpenCameraCalibrator - Insta360 Calibrator")
    parser.add_argument("-i", '--input_video', 
                        required=True,
                        help="Path to the mp4 video or the folder containing the mp4 videos")
    parser.add_argument('-k', '--keep_raw_telemetry',
                        action='store_true', 
                        help="Whether to keep the raw telemetry file or not")
    args = parser.parse_args()

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(path_to_file,"../")
    telemetry_conv = TelemetryConverter()

    video_files = []
    if args.input_video.endswith('.mp4'):
        video_files.append(args.input_video)
    else:
        video_files = glob.glob(pjoin(args.input_video, '*.mp4'))
    for video_file in video_files:
        cam_raw_telemetry = video_file.replace('.mp4', '_raw_telemetry.txt')
        cam_processed_telemetry = video_file.replace('.mp4', '_telemetry.json')
        with open(cam_raw_telemetry, 'w') as f:
            cam_extraction = Popen([
                f"{path_to_src}/Thirdparty/exiftool/exiftool", 
                "-m", "-ee", video_file
            ], stdout=f)
            cam_extraction.wait()
        telemetry_conv.convert_insta360_telemetry(video_file, cam_raw_telemetry, cam_processed_telemetry)
        if not args.keep_raw_telemetry:
            os.remove(cam_raw_telemetry)

if __name__ == "__main__":
    main()