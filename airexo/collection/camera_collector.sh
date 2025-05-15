source /home/ubuntu/mambaforge/etc/profile.d/conda.sh
conda activate airexo
export DISPLAY=:1
python -m airexo.collection.camera_collector --config-name=$1 +save_path=$2 +save_freq=$3 