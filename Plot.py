#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import websockets
import asyncio
import pandas as pd
import json
from datetime import datetime

class WebSocketToRosNode:
    def init(self):
        rospy.init_node('plotjuggler', anonymous=True)

        # Define the ROS 1 publisher
        self.publishers = {}

        # Initialize Pandas DataFrame and CSV file for logging
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = 'log_plot_data_{}.csv'.format(self.timestamp)
        self.df = pd.DataFrame(columns=['timestamp', 'key', 'value'])

        # Start the WebSocket server
        self.loop = asyncio.get_event_loop()
        start_server = websockets.serve(self.websocket_handler, "localhost", 5000)
        self.loop.run_until_complete(start_server)
        self.loop.run_forever()

    async def websocket_handler(self, websocket, path):
        async for message in websocket:
            rospy.loginfo("Received message: {}".format(message))
            self.process_data(message)

    def process_data(self, data):
        try:
            # Parse JSON data
            json_data = json.loads(data)
            rospy.loginfo("Parsed JSON data: {}".format(json_data))

            # Extract key and value
            for key, value in json_data.items():
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                # Log data to DataFrame
                new_row = pd.DataFrame({'timestamp': [timestamp], 'key': [key], 'value': [value]})
                self.df = pd.concat([self.df, new_row], ignore_index=True)

                # Publish data to ROS 1 topic
                if key not in self.publishers:
                    self.publishers[key] = rospy.Publisher('plot_data/{}'.format(key), String, queue_size=10)

                msg = String()
                msg.data = '{}: {}'.format(key, value)
                self.publishers[key].publish(msg)
                rospy.loginfo("Publishing to plot_data/{}: {}".format(key, value))

                # Save DataFrame to CSV file
                if not self.df.empty:
                    self.df.to_csv(self.csv_filename, index=False)
                    rospy.loginfo("DataFrame saved to {}".format(self.csv_filename))
                else:
                    rospy.logwarn("DataFrame is empty; no data to save.")

        except json.JSONDecodeError:
            rospy.logerr('Failed to decode JSON: {}'.format(data))
        except Exception as e:
            rospy.logerr('Error processing data: {}'.format(str(e)))

if name == 'main':
    try:
        WebSocketToRosNode()
    except rospy.ROSInterruptException:
        pass