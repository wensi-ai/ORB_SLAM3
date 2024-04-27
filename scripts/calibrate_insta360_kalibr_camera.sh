DOWNSAMPLE_FAC=2

BASEPATH=${HOME}/Documents/Files/SLAM/insta
DATASET=kalibr_1080_linear


INPUT=${BASEPATH}/${DATASET}/cam/cam.mp4
python scripts/insta360_extract_for_kalibr.py --input_video=${INPUT} --skip_frames=4 --downsample_fac=${DOWNSAMPLE_FAC}

xhost +local:root
CONTAINER_ID=$(docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --rm --detach -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ${BASEPATH}/${DATASET}:/data kalibr)

docker exec -it ${CONTAINER_ID} bash -c ' \
    source /catkin_ws/devel/setup.bash; \
    rosrun kalibr kalibr_bagcreater --folder=/data/cam/bag_input --output-bag=/data/cam/for_kalibr.bag; \
    rosrun kalibr kalibr_calibrate_cameras --bag=/data/cam/for_kalibr.bag --models pinhole-radtan --target /data/kalibr_apriltag.yaml --topics /cam0/image_raw --dont-show-report --show-extraction;'