Instructions for setting up ROVIO

-Install ROVIO (https://github.com/ethz-asl/rovio) in your Catkin workspace. Don't forget to build it.

-Create symlinks in the ROVIO installation and the yaml, info and launch files included in this folder.
e.g. (executed in the parent folder of this file (rovio)) ln -s "$(pwd)/airsim_camera.yaml" ~/catkin_ws/src/rovio/cfg (or rovio/launch for the launch file)
The "$(pwd)" part of the command gets the absolute path to your current working directory, the rest appends the file we want to link.
You can also type out the full path, in my case: ln -s ~/vtolbook/airsim/rovio/airsim_camera.yaml ~/catkin_ws/src/rovio/cfg 
I recommend checking the link by cd into the rovio/cfg folder and running: ls -l airsim_camera.yaml
The output should be something like this: 

lrwxrwxrwx 1 james james 58 Apr 17 17:41 rovio_airsim_node.launch -> /home/james/vtolbook/airsim/rovio/rovio_airsim_node.launch

If the link is not red, or it has the full path after the arrow, you're good to go!

-Link (or copy) settings.json file into your Documents folder (this version is different than the one in the settings folder)