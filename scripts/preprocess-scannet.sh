id=(0000 0059 0106 0169 0181 0207)
data_dir=../data/ScanNet

download_dataset () {
    echo "Downloading dataset... This may take a long time..."
    for i in ${id[@]}; do
        yes "" | python3 download-scannet.py --out_dir $data_dir --id scene${i}_00 --type .sens
    done
}

extract_sens () {
    echo "Extracting from Sens... This may take up to 10 minutes..."
    for i in ${id[@]}; do
        python3 reader.py --filename ../data/ScanNet/scans/scene${i}_00/scene${i}_00.sens --output_path ../data/ScanNet/scans/scene${i}_00/ --export_depth_images --export_color_images --export_poses
    done
    wait
}

convert_pose () {
    id=$1
    pose_dir=${data_dir}/scans/scene${id}_00/pose
    pose_file=${data_dir}/scans/scene${id}_00/trajectory.txt
    rm -f $pose_file
    for i in `ls $pose_dir | sort -k1 -n`; do
        echo ${i%.*} $(sed -z 's/\n/\ /g' $pose_dir/$i) >> $pose_file
    done
}

if [ ! -f download-scannet.py ]; then echo 'Go to https://github.com/ScanNet/ScanNet to ask for authorization.'; exit 1; fi
pip3 install -r requirements.txt
mkdir -p ${data_dir}
download_dataset
extract_sens
for i in ${id[@]}; do
    convert_pose $i
done
