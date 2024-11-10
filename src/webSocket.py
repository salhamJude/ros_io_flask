#!/usr/bin/env python3

import flask
import socketio
import rospy
from std_msgs.msg import String, Bool, ByteMultiArray, Int32
from flask_cors import CORS


class WebSocket:
    def __init__(self):
        self.app = flask.Flask(__name__)
        
        # Initialize CORS with wildcard to allow any origin
        CORS(self.app, resources={r"/*": {"origins": "*"}})

        # Manually add CORS headers after each request
        @self.app.after_request
        def apply_cors(response):
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
            return response

        # Initialize SocketIO with CORS allowed
        self.sio = socketio.Server(cors_allowed_origins="*")
        self.app.wsgi_app = socketio.WSGIApp(self.sio, self.app.wsgi_app)

        # Define your publishers and subscribers here
            #self.cam_sub = rospy.Subscriber("camera_feed", ByteMultiArray, self.cameraFeedCallback)

        # Define Register event handlers here
        self.sio.on("connect", self.onConnect)
        self.sio.on("disconnect", self.onDisconnect)


    # def cameraFeedCallback(self,data):
    #     pass

    def onConnect(self, sid, environ):
        print(f"client {sid} connected")
    
    def onDisconnect(self, sid):
        print(f"client {sid} disconnected")




    def connect(self):
        self.app.run(host="0.0.0.0", port=5000)

    def message(self, sid, message):
        print(f"client {sid} : {message}")

if __name__ == '__main__':
    from threading import Thread

    #run the server on main thread and ros on a second(reverse cause the server not to run properly)
    Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
    server = WebSocket()
    server.connect()
    rospy.spin()
