

## For robot with base
catkin build
export ROBOT_MODEL="xarm6_box_base2_ee"

export ROBOT_XACRO=`rospack find xarm_description`/urdf/with_base/"$ROBOT_MODEL".urdf.xacro
export ROBOT_OUT=`rospack find xarm_description`/robots/"$ROBOT_MODEL"

rosrun xacro xacro $ROBOT_XACRO > "$ROBOT_OUT".urdf

#export ROBOT_MODEL="xarm6_box_base2"
export ROBOT_XACRO=`rospack find xarm_description`/urdf/with_base/"$ROBOT_MODEL".urdf.xacro
export ROBOT_OUT=`rospack find xarm_description`/urdf/with_base/"$ROBOT_MODEL"

rosrun xacro xacro $ROBOT_XACRO > "$ROBOT_OUT".urdf
rosrun collada_urdf urdf_to_collada "$ROBOT_OUT".urdf "$ROBOT_OUT".dae
openrave "$ROBOT_OUT".dae


export ROBOT_PATH=`rospack find xarm_description`/urdf/with_base/
export MYROBOT_NAME=$ROBOT_PATH"$ROBOT_MODEL"
export BASE_LINK="0"
export EEF_LINK="13"

export IKFAST_OUTPUT_PATH=`rospack find map_creator`/include/map_creator/ikfast61_"$ROBOT_MODEL".cpp

export IKFAST_PRECISION="5"
cp "$MYROBOT_NAME".dae "$MYROBOT_NAME".backup.dae  # create a backup of your full precision dae.
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"

python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py \
--robot="$MYROBOT_NAME".dae \
--iktype=transform6d \
--baselink="$BASE_LINK" \
--eelink="$EEF_LINK" \
--savefile="$IKFAST_OUTPUT_PATH"

