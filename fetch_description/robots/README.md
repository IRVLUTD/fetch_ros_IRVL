# Fetch robot models

| File | End effector | Calibration |
| --- | --- | --- |
| `fetch.urdf` | ATI sensor and LEAP hand | Nominal Fetch geometry |
| `fetch_ati_sensor.urdf` | ATI sensor and UMI fingers | Nominal Fetch geometry |
| `fetch_original_fingers.urdf` | ATI sensor and original fingers | Nominal Fetch geometry |
| `calibrated/calibrated_2024_12_09_20_00_06.urdf` | ATI sensor and LEAP hand | December 2024 Fetch calibration |
| `calibrated/calibrated_2024_12_09_20_00_06_ati_sensor.urdf` | ATI sensor and UMI fingers | December 2024 Fetch calibration |
| `calibrated/calibrated_2024_12_09_20_00_06_original_fingers.urdf` | ATI sensor and original fingers | December 2024 Fetch calibration |

The LEAP variants in `fetch.urdf` and the calibrated file preserve their
corresponding Fetch model, including the corrected `ati_axis` roll of
`2.31289265` radians. Their hand geometry, joints, and `ati_to_leap` mounting
transform were copied from the former `fetch_leap.urdf`.
The calibrated LEAP variant reuses the existing Fetch calibration; it is not
the result of a new calibration with the LEAP hand installed.

The calibrated LEAP model also retains the `gripper_link` endpoint required
by Fetch's installed gravity-compensation configuration. This frame coincides
with `palm_lower`; the existing ATI-to-palm transform is unchanged. All 17
LEAP palm/finger inertias are combined into a single 0.894 kg payload on
`gripper_link`, using the finger joints at zero radians. Original distributed
inertias remain in XML comments and are not counted again on the articulated
links. This is a fixed-payload approximation based on the existing URDF data,
not a measurement of the installed hand/adapter. Finger motion does not update
its center of mass. Do not use this lumped model for accurate finger dynamics.

The nominal `fetch.urdf` retains the distributed LEAP inertias and is intended
for visualization, planning experiments, and other uses that do not require
Fetch's hardware-controller endpoint. It lacks `gripper_link`: **do not copy
it into Fetch's hardware startup configuration**. The installed gravity
controller expects that link and will fail to initialize without it. For
LEAP hardware startup, use the calibrated LEAP file listed above.

`fetch_moveit_config/launch/moveit.launch` selects the calibrated LEAP model
with `robot:=fetch_leap`, and the saved calibrated UMI model with
`robot:=fetch_UMI` (the default). `robot:=fetch_original` selects the calibrated
original-finger model.

These workspace files are separate from the installed URDF under `/etc/ros`.
Editing them does not update the robot's startup copy or running ROS nodes.

## Permanent startup on Fetch

On this robot, `/etc/ros/melodic/robot.launch` loads
`/etc/ros/melodic/calibrated_2024_12_09_20_00_06.urdf`. Keep the existing launch
and TF publisher; manually replace only that installed URDF. Check the
`robot_description` textfile path in your robot's launch before using these
commands if its installation differs. No files under `/opt/ros` need changing.

Run on Fetch after copying/syncing the workspace edits there:

```bash
cd ~/Projects/uhas
source devel/setup.bash

URDF_FILE="$(rospack find fetch_description)/robots/calibrated/calibrated_2024_12_09_20_00_06.urdf"
INSTALLED_FILE=/etc/ros/melodic/calibrated_2024_12_09_20_00_06.urdf
BACKUP_FILE="${INSTALLED_FILE}.backup-$(date +%Y%m%d-%H%M%S)"

# Copy only after validation and backup succeed.
if check_urdf "$URDF_FILE"; then
  sudo cp -a "$INSTALLED_FILE" "$BACKUP_FILE" &&
    sudo cp "$URDF_FILE" "$INSTALLED_FILE"
fi
```

The copy does not reload parameters, restart services, or change the running
controllers. It takes effect at the next normal robot bringup. Support/tuck
the arm before any full service restart or reboot because hardware drivers
restart too. After startup, check that gravity compensation initialized
successfully before commanding arm or torso movement. Displaying the model
in RViz alone does not verify that the hardware controllers initialized.

The boot TF publisher reads the installed URDF without loading its meshes.
Source the workspace containing `fetch_description` and `leap_description`
in each terminal before starting RViz or MoveIt, so those programs can load
visual and collision meshes. The copy is a snapshot: repeat it after changes
that should also apply at boot.

To switch back to UMI or original fingers, select the corresponding calibrated
file from the table as `URDF_FILE`, then validate, back up, and copy it to the
same `INSTALLED_FILE` path. Use a model matching the mounted hardware and
select the matching MoveIt configuration. Other hands need a compatible
hardware-controller endpoint and appropriate payload inertias as well as
matching geometry; do not install an arbitrary visualization-only URDF.

To restore a previous copy, substitute the exact backup filename:

```bash
sudo cp -a /etc/ros/melodic/calibrated_2024_12_09_20_00_06.urdf.backup-TIMESTAMP \
  /etc/ros/melodic/calibrated_2024_12_09_20_00_06.urdf
```

Restoring the file also takes effect at the next normal robot bringup.

## LEAP feedback and MoveIt

`uhas_fetch_leap.launch` now starts a `topic_tools relay` from its
`leap_state_topic` (default `/leap_hand/state`) to `/joint_states`. This forwards
real feedback, not commanded positions or placeholder states. The existing
Fetch publisher continues supplying Fetch joints on `/joint_states`.
The relay respawns automatically. Stop any manually started relay before
launching this version. If another component already forwards hand feedback
(for example the LEAP trajectory action server), pass
`publish_joint_states:=false` to avoid duplicate forwarding. If
`leap_state_topic:=/joint_states`, the relay is automatically skipped.

Start the hand/control launch in its usual sourced Python environment:

```bash
roslaunch leap_ethernet uhas_fetch_leap.launch
```

This retains the current explicit startup of the hand and UHAS controller;
it does not auto-start hand torque or arm command processing at boot.

When using MoveIt, select the same robot as the installed boot model:

```bash
roslaunch fetch_moveit_config moveit.launch robot:=fetch_leap
```

That launch reloads a workspace URDF. After subsequent edits, refresh the
installed URDF copy too so boot and planning use the same model. A custom hand
also requires matching MoveIt SRDF/controller configuration; selecting a URDF
alone does not configure a new hand controller.
