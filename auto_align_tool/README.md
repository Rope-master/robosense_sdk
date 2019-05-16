#auto_align_tool (lidar mounting calibration tool):

- **What is this tool for ?**
This is an auxiliary calibration tool for lidar mounting configuration parameters, i.e., x, y, z, roll, pith, yaw of lidar mounted relative to the mother vehicle. The main purpose of building this tool is to help users from the laborious work of measuring those parameters by hand. From pespective of algorithm, calibrating those parameters is a useful preprocesing which translate the various application situations to a relatively "normalized space" that can span the applicable range of the SDK. **In one word, this tool is designed to estimate the lidar mounting height and orientation (x-axis of the local lidar coordinates) and keep the ground points horizontal.**

- **How to use ?**
Finding an open flat place (length or width > 15 m) , mounting the lidar to the mother vehicle and keep the lidar fixed tightly. Receive data and make the calibration in real time or collecting data in bag format and make calibration offline. The data source can be configured in launch file. **Notice that this tool should be used with company of lidar driver package (`play_tool`)** to get original data parsed.

Launch the tool by:
```
cd Robosense_sdk
roslaunch auto_align_tool lidar_calibration.launch
```
you will see:

![image](http://qncdn.sz-sti.com/autoalign/aligntool.png)

The purple transparent plane is the virtul standard ground reference, the cyan box is the virtual vehicle, the axies (red is x, green is y, blue is z) is the vehicle reference coordinate system (see below for detail), and the yellow small box and arraw are the virtual lidar and its orientation. Users can calibrate the lidar mounting parameters through translation and rotation by the contral panel:

![image](http://qncdn.sz-sti.com/autoalign/align_panel.png)

Recommended way:
1) set the vehicle size first according to your own testing platform;
2) adjust the lidar height (`set trans_z`);
3) adjust the lidar position (`set trans_x` and `set trans_y`);
4) adjust the yaw angle (`set Rotation Yaw`);
5) adjust the pointcloud to keep ground points hided in the purple plane through adjusting the roll and pitch angle (`set Rotation Roll` and `set Rotation Pitch`).

Lidar height and position is measured relative to vehicle coordinate, i.e., trans_x, trans_y, trans_z are lidar's position coordiantes under vehicle coordinate system, so as the roll, pitch, yaw. Commonly, yaw angle and  lidar height are easily to confirm, most effort will be cost on adjusting the ground plane to be aligned horizontally. **Keep adjusting until most of the ground points are aligned horizantally and hided to the reference purple plane (may need adjusting the lidar height) and with right relative angle between lidar and vehicle**.

When calibration done, click `save` button to save the result and then the following perception procedure can query the calibrated mounting parameters automatically through the path configuration in launch file. If user wants to recalibrate from the last result, click the `load` button to recover the last result and going on.