source scripts/init.sh
python fast_lane_detection.py
if [ $? -ne 0 ]; then
    printf "\nLane detection failed. Make sure you have the CARLA server on. Exiting...\n"
    exit 1
fi
printf "\nData collecting done. Generating dataset...\n"
python dataset_generator.py 
printf "\nFinished generating dataset.\n"
