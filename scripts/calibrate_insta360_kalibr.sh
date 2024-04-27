# steps how to run kalibr
# 1. docker pull stereolabs/kalibr
# 2. Use python/extract_for_kalibr_bagcreator.py to extract telemetry and single images
# 3. Kalibr calibrate camera
# 4. Kalibr calibrate cam imu

DOWNSAMPLE_FAC=2

BASEPATH=/home/svl/Documents/Files/SLAM/insta
DATASET=kalibr_1080_fisheye


INPUT=${BASEPATH}/${DATASET}/cam/cam.mp4
python3 ../python/insta360_extract_for_kalibr.py --input_video=${INPUT} --skip_frames=4 --downsample_fac=${DOWNSAMPLE_FAC}

INPUT=${BASEPATH}/${DATASET}/cam_imu/cam_imu.mp4
python3 ../python/insta360_extract_for_kalibr.py --input_video=${INPUT} --skip_frames=1 --downsample_fac=${DOWNSAMPLE_FAC}

xhost +local:root
CONTAINER_ID=$(docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --rm --detach -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ${BASEPATH}/${DATASET}:/data kalibr)

docker exec ${CONTAINER_ID} bash -c ' \
    source /catkin_ws/devel/setup.bash; \
    rosrun kalibr kalibr_bagcreater --folder=/data/cam/bag_input --output-bag=/data/cam/for_kalibr.bag; \
    rosrun kalibr kalibr_bagcreater --folder=/data/cam_imu/bag_input --output-bag=/data/cam_imu/for_kalibr.bag; \
    rosrun kalibr kalibr_calibrate_cameras \
        --bag=/data/cam/for_kalibr.bag --models pinhole-equi \
        --target /data/kalibr_apriltag.yaml --topics /cam0/image_raw --dont-show-report; \
    rosrun kalibr kalibr_calibrate_imu_camera \
        --target /data/kalibr_apriltag.yaml --imu /data/imu_noise.yaml \
        --cam /data/cam/for_kalibr-camchain.yaml \
        --bag /data/cam_imu/for_kalibr.bag --dont-show-report;'