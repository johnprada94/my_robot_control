#!/usr/bin/env python3

import rospy
import socket
import signal
import sys
from threading import Thread
from std_msgs.msg import String

# List to keep track of client threads
client_threads = []

# IP address from which data will be processed
target_ip = '10.30.1.63'

# Global variable to hold the latest joint positions
latest_joint_positions = None

# Function to handle the subscriber callback
def joint_positions_callback(msg):
    global latest_joint_positions
    latest_joint_positions = msg.data

# Function to handle each client connection
def handle_client(client_socket, client_address, joint_publisher, transform_publisher):
    rospy.loginfo(f"Connection established with {client_address}")
    try:
        while not rospy.is_shutdown():
            # Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            if not data:
                break
            rospy.loginfo(f"Received from {client_address}: {data}")

            # Check if the message is from the target IP address
            if client_address[0] == target_ip:
                values = data.split(',')

                # Ensure we have exactly 7 values (6 joint positions and 1 index)
                if len(values) == 7:
                    joint_positions = values[:6]  # First 6 values are joint positions
                    index = int(values[6])  # The 7th value is the index

                    # Check the index value
                    if index == 1:
                        # Convert joint positions to a string and publish to the ROS topic
                        joint_data = ','.join(joint_positions)
                        joint_publisher.publish(joint_data)
                        rospy.loginfo(f"Published joint positions: {joint_data}")
                        client_socket.send("Acknowledged".encode('utf-8'))

                    elif index == 0:
                        if latest_joint_positions is not None:
                            client_socket.send(latest_joint_positions.encode('utf-8'))
                        else:
                            rospy.loginfo("No joint positions available from /ur10_joint_positions_in_degrees")
                else:
                    rospy.logwarn("Received incorrect number of values")
            else:
                # Process transformation matrices for clients other than target IP
                if "Robot:" in data and "Object:" in data:
                    # Extract and publish the transformation matrices
                    transform_publisher.publish(data)
                    rospy.loginfo(f"Published transformation matrices:\n{data}")

            # Send a response back to the client
    except socket.error as e:
        rospy.logerr(f"Socket error: {e}")
    finally:
        client_socket.close()
        rospy.loginfo(f"Connection closed with {client_address}")

# Main server function
def tcp_server():
    rospy.init_node('tcp_server_node', anonymous=True)
    server_ip = '10.8.2.105'  # Listen on all available interfaces
    server_port = 5016  # Choose an appropriate port

    # Create ROS publishers
    joint_publisher = rospy.Publisher('/tcp_server_data', String, queue_size=10)
    transform_publisher = rospy.Publisher('/tcp_server_transforms', String, queue_size=10)

    # Create a ROS subscriber
    rospy.Subscriber('/ur10_joint_positions_in_degrees', String, joint_positions_callback)

    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(10)  # Allow up to 10 simultaneous connections
    rospy.loginfo(f"Server started on {server_ip}:{server_port}")

    # Accept clients until ROS is shut down
    while not rospy.is_shutdown():
        try:
            # Wait for a connection
            client_socket, client_address = server_socket.accept()
            rospy.loginfo(f"New connection from {client_address}")
            
            # Create a new thread to handle the client
            client_thread = Thread(target=handle_client, args=(client_socket, client_address, joint_publisher, transform_publisher))
            client_thread.start()
            client_threads.append(client_thread)
        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break

    # Close the server socket
    server_socket.close()
    rospy.loginfo("Server shut down")

    # Wait for all client threads to finish
    for thread in client_threads:
        thread.join()

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down server...")
    sys.exit(0)

if __name__ == '__main__':
    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass
