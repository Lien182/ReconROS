DATE=`date +%d-%m-%y_%H-%M`
NAME=${PWD##*/}
echo  "rm /upb/users/s/sheikh/profiles/unix/agce/ws/hw_build_${NAME}/ -r -f"
ssh sheikh@cc-9.cs.upb.de "rm /upb/users/s/sheikh/profiles/unix/agce/ws/hw_build_${NAME}/ -r -f"
rsync -r -v build.cfg build.msg src --exclude 'src/application' --exclude 'src/_build' sheikh@cc-9.cs.upb.de:/upb/users/s/sheikh/profiles/unix/agce/ws/hw_build_${NAME}
sshcommand="source /opt/ros/dashing/setup.bash; source /opt/Xilinx/Vivado/2017.1/settings64.sh; source bashinit; export XILINXD_LICENSE_FILE=27000@license5.uni-paderborn.de; source /upb/users/s/sheikh/profiles/unix/agce/ws/ReconROS/tools/settings.sh; cd /upb/users/s/sheikh/profiles/unix/agce/ws/hw_build_${NAME}/; rdk export_hw | tee rdk_export_hw.log; rdk build_hw | tee rdk_build_hw.log; exec bash"
ssh -t sheikh@cc-9.cs.upb.de "screen bash -c '" $sshcommand "'"
rsync -r -v sheikh@cc-9.cs.upb.de:/upb/users/s/sheikh/profiles/unix/agce/ws/hw_build_${NAME} hw_build_remote_${DATE}
