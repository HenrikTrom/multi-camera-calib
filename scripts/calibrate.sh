#!/bin/bash

W=1024
H=768

sleep 10

input_dir="../data/videos"
output_dir="../data/images"

rm -rf $input_dir/*.mp4
rm -rf $output_dir/*.png

/opt/modules/flirmulticamera/build/record_synchronized_videos ../cfg/record_video_Settings${W}x${H}.json 30

if [ -d "$input_dir" ];
then
    echo "Extracting frames to $output_dir"
else
	echo Error: "$input_dir directory does not exist."
    exit
fi

# Loop through .mp4 files in the directory
for video_file in "$input_dir"/*.mp4; do
    if [ -f "$video_file" ]; then
        # Extract the filename without extension
        filename=$(basename "$video_file" .mp4)
        
        ffmpeg -hide_banner -i $video_file "$output_dir/cam$filename-%4d.png"
        
        echo "Frames extracted for $video_file"
    fi
    
done

echo "Frame extraction complete"

# compute transform
../build/Multi_Camera_Calibration 
# test transform
python3 check_transform.py ${W} ${H}