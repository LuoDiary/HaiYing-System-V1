#!/usr/bin/env python3

import math
import os
import random
import time

os.environ.setdefault('MAVLINK20', '1')
os.environ.setdefault('MAVLINK_DIALECT', 'common')

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

PORT = 'udpout:127.0.0.1:14550'


def main():
    conn = mavutil.mavlink_connection(PORT, source_system=1, source_component=1)
    print(f'fake PX4 peer on {PORT} (ctrl-c to stop)')

    armed = False
    mode = 1
    start = time.time()
    last_beat = 0.0
    last_att = 0.0
    last_vib = 0.0
    last_pos = 0.0

    pos = [0.0, 0.0, -1.0]
    vel = [0.0, 0.0]

    while True:
        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            t = msg.get_type()
            line = 'RX ' + t
            if t == 'SET_ATTITUDE_TARGET':
                line += ' q=' + str([round(float(x), 4) for x in msg.q])
                line += ' thrust=' + format(msg.thrust, '.4f')
                line += ' mask=' + str(msg.type_mask)
                line += ' sys=' + str(msg.target_system) + ' comp=' + str(msg.target_component)
            elif t == 'COMMAND_LONG':
                line += ' cmd=' + str(msg.command)
                line += ' p1=' + format(msg.param1, '.2f') + ' p2=' + format(msg.param2, '.2f')
                if msg.command == mavlink.MAV_CMD_DO_SET_MODE:
                    if int(msg.param2) == 6:
                        mode = 6
                    elif int(msg.param2) == 3:
                        mode = 3
                    elif int(msg.param2) == 4:
                        if int(msg.param3) == 3:
                            mode = 4
                        elif int(msg.param3) == 5:
                            mode = 5
                        elif int(msg.param3) == 6:
                            mode = 12
                        else:
                            mode = 4
                    elif int(msg.param2) == 1:
                        mode = 1
                    conn.mav.command_ack_send(msg.command, 0, 0, 0, 0, 0)
                elif msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    armed = bool(int(msg.param1))
                    conn.mav.command_ack_send(msg.command, 0, 0, 0, 0, 0)
            print(line, flush=True)

        now = time.time()
        if now - last_beat >= 0.5:
            last_beat = now
            base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if armed:
                base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            conn.mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_PX4,
                base_mode, mode << 16, mavlink.MAV_STATE_ACTIVE)
        if now - last_att >= 0.1:
            last_att = now
            conn.mav.attitude_quaternion_send(
                int((now - start) * 1000) % 2**32,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0)
        if now - last_vib >= 0.5:
            last_vib = now
            conn.mav.vibration_send(
                int(now * 1e6) % 2**64,
                4.0 + 1.5 * math.sin(now * 0.7),
                3.0 + 1.0 * math.sin(now * 0.5 + 1.0),
                7.0 + 2.0 * math.sin(now * 0.9 + 2.0),
                0, 0, 0)
        if now - last_pos >= 0.1:
            last_pos = now
            vel[0] += random.uniform(-0.006, 0.006)
            vel[1] += random.uniform(-0.006, 0.006)
            vel[0] = max(-0.04, min(0.04, vel[0]))
            vel[1] = max(-0.04, min(0.04, vel[1]))
            pos[0] += vel[0] * 0.1
            pos[1] += vel[1] * 0.1
            conn.mav.local_position_ned_send(
                int((now - start) * 1000) % 2**32,
                pos[0], pos[1], pos[2],
                vel[0], vel[1], 0.0)


if __name__ == '__main__':
    main()
