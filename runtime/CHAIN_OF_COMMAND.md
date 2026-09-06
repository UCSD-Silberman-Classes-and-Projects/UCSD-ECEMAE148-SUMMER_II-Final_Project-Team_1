# MANTA Integrated Chain of Command

## Vehicle Command Chain

GPS + IMU
        |
        v
manta_localization
        |
        v
/manta/fused_odom
        |
        v
waypoint_controller
        |
        v
/manta/nav/cmd_vel_raw
        |
        v
manta_safety_gate
        |
        v
/cmd_vel
        |
        v
vesc_twist_node
        |
        v
Vehicle


## Safety Inputs

LD06
  |
  v
/scan
  |
  v
manta_lidar_adapter
  |
  +--------------------+
                       |
                       v
                manta_safety_gate
                       ^
                       |
OAK-D ----------------+


## Command Ownership

Navigation owns:

    /manta/nav/cmd_vel_raw

Navigation must NOT publish directly to:

    /cmd_vel


Safety gate owns:

    /cmd_vel


VESC consumes:

    /cmd_vel


## Localization Ownership

GPS and IMU feed:

    manta_localization

Localization owns:

    /manta/fused_odom


LiDAR and OAK-D are safety/perception inputs.

They do NOT provide global position or navigation yaw.


## Integration Rule

Exactly ONE waypoint_controller may run.

Do not bypass manta_safety_gate during integrated operation.

Do not modify teammate-owned component source code through the
integration layer.

manta_runtime only launches and monitors existing components.
