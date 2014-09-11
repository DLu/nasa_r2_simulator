r2_gazebo_dir=`rospack find r2_gazebo`
config_directories=`find $r2_gazebo_dir -mindepth 1 -maxdepth 1 -type d \( ! -iname ".git" \)`

for d in $config_directories; do
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$d/
done

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${r2_gazebo_dir}/Media/models