# syringe_ui

PySide2-based ROS 2 UI node for titration experiments:
- Node name: `syringe_ui`
- Subscribes: `/ph` (std_msgs/Float32), `/temperature` (std_msgs/Float32), `/titration_vol` (std_msgs/UInt32)
- Publishes: `/syringe_cmd` (std_msgs/Int8) via UI buttons.
- Real-time plot: pH vs titration volume (Y: 0..14, X: 0..max volume).
- Save Data: writes a timestamped CSV with columns `Time, pH, Temperature, Titration volume`
- Modify launch file for the data output directory.

## Installaion requirement

```bash
pip install setuptools==66.1.1
sudo apt update
sudo apt install python3-pyside2.qtcore
sudo apt install python3-pyside2.qtgui
sudo apt install python3-pyside2.qtwidgets
sudo apt install python3-pyside2.qtuitools
# Then
sud0 apt install python3-pyqtgraph
# Or
pip install pyqtgraph
```

## Run

After build
```bash
ros2 launch syringe_ui syringe_ui.launch.py 

```
The window will then allow dispensing small volume for test. 
Click `Start plot and save data` before beginning the automatic titration using syringe pump.
When finished, click `Stop saving` and `Open AI assistant` to begin data analysis.
