#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import datetime

rospack = rospkg.RosPack()
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.scriptapi.ROSLaunch()
launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('ti_mmwave_rospkg')+"/launch/1843es1_short_range.launch",
                                                        rospack.get_path('zed_wrapper')+"/launch/zed.launch"])
launch.start()

# Launch mmWaveQuickConfig node
mmWaveQuickConfigNode = roslaunch.core.Node("ti_mmwave_rospkg",
                                            "mmWaveQuickConfig",
                                            name="mmWaveQuickConfig",
                                            args=rospack.get_path('ti_mmwave_rospkg')+"/cfg/1843es1_short_range_3d.cfg",
                                            output="screen")
process = launch.launch(mmWaveQuickConfigNode)

#Wait for mmWaveQuickConfig node to finish
while process.is_alive():
    rospy.sleep(1)

#Check if mmWaveQuickConfig node finished properly
desc = process.get_exit_description()
code_loc = desc.find("exit code")
if (desc[code_loc+10:code_loc+11] == "1"):
    rospy.logerr("Closing launch file!!, mmWaveQuickConfig node failed to execute properly")
    launch.stop()
else:
    #Launch recording node
    recordingNode = roslaunch.core.Node("ti_mmwave_rospkg",
                                        "mmWaveDataRecorder",
                                        name="mmWaveDataRecorder",
                                        output="screen")
    recordingProcess = launch.launch(recordingNode)

    #Wait for recording node to finish
    while recordingProcess.is_alive():
        rospy.sleep(1)

launch.stop()
