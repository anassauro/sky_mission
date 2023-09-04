# import time
# from pymavlink import mavutil

# master = mavutil.mavlink_connection('udp:localhost:14550')

# # Wait for the heartbeat message to confirm the connection
# master.wait_heartbeat()
# print("Connected to system:", master.target_system, ", component:", master.target_component)
# while True:
#     # Send a status text message with severity info
#     text = "Hello Pixhawk".encode('utf-8')
#     master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

#     # Wait a bit to give Pixhawk time to process the message
#     time.sleep(1)
# wait for a heartbeat
# inform user
# print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)


# from pymavlink import mavutil

# # Start a connection listening on a UDP port
# the_connection = mavutil.mavlink_connection('udp:localhost:14550')

# # Wait for the first heartbeat 
# #   This sets the system and component ID of remote system for the link
# the_connection.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# # Once connected, use 'the_connection' to get and send messages

# """
#     If you have a companion computer on your vehicle, you can send a status text
#     message from the companion computer to the GCS using the MAVLink protocol to
#     inform the user of the status of the process running on the companion computer.

#     This is useful for debugging and for creating and sending messages to the GCS
#     that are not part of the standard MAVLink protocol. And also the companion
#     computer uses the MAVLink protocol and underlying infrastructure.

#     This example assumes that you have a companion computer on your vehicle and
#     that you have a UDP connection between the companion computer and the GCS.

#     Device string for UDP connection: "udpout:127.0.0.1:14560" means that the
#     companion computer acts as a source component (like the autopilot on the
#     MAVLink connection) and sends messages to the GCS on the local machine.

#     At the ardu-sim.sh, replace the "--out 127.0.0.1:14560" with "--master 127.0.0.1:14560"

#     https://mavlink.io/en/messages/common.html#STATUSTEXT
# """

# import time
# import random
# import pymavlink.mavutil as utility
# import pymavlink.dialects.v20.all as dialect

# # create a source component
# my_process = utility.mavlink_connection(device="udpout:127.0.0.1:14550",
#                                         source_system=1,
#                                         source_component=1)

# # inform user
# print("Serving as a system:", my_process.source_system, ", component:", my_process.source_component)

# # create an infinite loop
# while True:

#     # create the text
#     text = f"2 people detected: 1 lying 1 standing"

#     # create STATUSTEXT message
#     message = dialect.MAVLink_statustext_message(severity=dialect.MAV_SEVERITY_INFO,
#                                                  text=text.encode("utf-8"))

#     # send message to the GCS
#     my_process.mav.send(message)

#     # sleep a bit
#     time.sleep(5)

"""
    Source: https://mavlink.io/en/messages/common.html#COMMAND_LONG
            https://mavlink.io/en/messages/common.html#MAV_CMD
            https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER
            https://mavlink.io/en/messages/common.html#STATUSTEXT
            https://mavlink.io/en/messages/common.html#COMMAND_ACK

    To receive a message from MAVLink stream, recv_match() is used
    To send a message, mav.send() is used

    There are three ways to send commands to vehicle:
        COMMAND_LONG (MAV_CMD_* command messages)
        COMMAND_INT (MAV_CMD_* command messages)
        CUSTOM (Like SET_POSITION_TARGET_LOCAL_NED, SET_POSITION_TARGET_GLOBAL_INT, ...)
"""

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14551")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# build up the command message
message = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_SEND_BANNER,
                                               confirmation=0,
                                               param1=0,
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send the command to the vehicle
vehicle.mav.send(message)

# do below always
while True:

    # receive a message
    message = vehicle.recv_match(type=[dialect.MAVLink_statustext_message.msgname,
                                       dialect.MAVLink_command_ack_message.msgname],
                                 blocking=True)

    # convert message to dictionary
    message = message.to_dict()

    # show the message
    print(message)