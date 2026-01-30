# titration_robot_syringe

Requires the package "titration_robot_interfaces". 
- Servers: 
`/delta_ph` (titration_robot_interfaces/srv/DeltaPh)
`/derivative_ph_volume` (itration_robot_inerfaces/srv/DerivativePhVolume)
`/syringe` (titration_robot_interfaces/srv/Syringe) 
Syringe will initialize at 5 ml holding volume.

- Behavior Tree executor:
`syringe_equivalence_executor`

- Custom BT nodes:
`aspire_action` (not in use)
`check_dph_dv_condition`
`check_ph_by_range_condition`
`dispense_action`
`is_ph_stable_condition`
`run_once_decorator` (not in use)
`set_dispense_volume_action`

- Behavior Tree:
`syringe_equivalence.xml`


## Installaion requirement

```bash
sudo apt install ros-galactic-behaviortree-cpp-v3
```

## Run

After build
```bash
# First bring up the servers
ros2 launch titration_robot_syringe titration_server.launch.py 

# Then, in another terminal:
ros2 launch titration_robot_syringe syringe_equivalence_executor.launch.py 
```
The BT will then automate the process until `check_ph_by_range_condition` is satisfied.