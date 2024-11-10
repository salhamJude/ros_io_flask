#!/usr/bin/env python3

import flask
import socketio
import rospy
from std_msgs.msg import String, Bool, ByteMultiArray, Int32
from flask_cors import CORS


class WebSocket:
    """
    Base WebSocket server using Flask, SocketIO, and ROS.
    This serves as a template for projects integrating web sockets with ROS communication.
    """
    def __init__(self):
        # Initialize Flask app and CORS to allow any origin
        self.app = flask.Flask(__name__)
        CORS(self.app, resources={r"/*": {"origins": "*"}})

        # Add CORS headers after each request
        @self.app.after_request
        def apply_cors(response):
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
            return response

        # Initialize SocketIO with CORS allowed
        self.sio = socketio.Server(cors_allowed_origins="*")
        self.app.wsgi_app = socketio.WSGIApp(self.sio, self.app.wsgi_app)

        # Publishers and subscribers should be defined here as needed
        # Example: self.cam_sub = rospy.Subscriber("camera_feed", ByteMultiArray, self.cameraFeedCallback)

        # Register event handlers for client connections
        self.sio.on("connect", self.onConnect)
        self.sio.on("disconnect", self.onDisconnect)

    # Define any ROS callback methods here
    # Example:
    # def cameraFeedCallback(self, data):
    #     pass

    def onConnect(self, sid, environ):
        """Handler for client connection event."""
        print(f"client {sid} connected")
    
    def onDisconnect(self, sid):
        """Handler for client disconnection event."""
        print(f"client {sid} disconnected")

    def connect(self):
        """Start the Flask app server."""
        self.app.run(host="0.0.0.0", port=5000)

    def message(self, sid, message):
        """Handler for incoming messages from clients."""
        print(f"client {sid} : {message}")

if __name__ == '__main__':
    from threading import Thread

    # Run the ROS node in a separate thread to allow server to start correctly
    Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
    
    # Initialize and start the WebSocket server
    server = WebSocket()
    server.connect()
    rospy.spin()
