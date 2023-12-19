import rclpy
from geometry_msgs.msg import Twist

from flask import Flask, render_template, send_from_directory, Response
from flask_sock import Sock
import socket
import threading
import cv2


class WebTeleop:
    def __init__(self):
        self.app = Flask(__name__)
        self.sock = Sock(self.app)
        
        self.node = rclpy.create_node('web_teleop')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

        #self.speed = 0.5

        @self.app.route('/')
        def index():
            hostname = (socket.gethostname().split(".")[0]).upper()
            return render_template('index.html', hostname=hostname)

        @self.app.route("/manifest.json")
        def manifest():
            return send_from_directory('./static', 'manifest.json')

        @self.app.route("/app.js")
        def script():
            return send_from_directory('./static', 'app.js')

        @self.sock.route('/command')
        def command(sock):
            while True:
                # Split the received command by ':' to get speed
                cmd = sock.receive().split(':')

                if cmd[0] == "left":
                    self.publish_cmd_vel(0.0, 1.0)

                elif cmd[0] == "right":
                    self.publish_cmd_vel(0.0, -1.0)

                elif cmd[0] == "up":
                    self.publish_cmd_vel(1.0, 0.0)

                elif cmd[0] == "down":
                    self.publish_cmd_vel(-1.0, 0.0)

                elif cmd[0] == "stop":
                    self.publish_cmd_vel(0.0, 0.0)

                #elif cmd[0] == "speed":
                #    self.speed = float(cmd[1])

                else: 
                    print("send either `up` `down` `left` `right` or `stop` to move your robot!")

        def video_gen(self):
            """Video streaming generator function."""
            while True:
                img = 0

                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                ret, jpeg = cv2.imencode('.jpg', img)
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route. Put this in the src attribute of an img tag."""
            return Response(video_gen(self), mimetype='multipart/x-mixed-replace; boundary=frame')
                    
    def publish_cmd_vel(self, linear_x, angular_z):
        # Create a Twist message and publish it
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.publisher.publish(twist_msg)

def main(args=None):
    print('Starting web_teleop.py.')

    rclpy.init(args=args)

    controller = WebTeleop()
    
    # Create threads for Flask app and ROS 2 node
    flask_thread = threading.Thread(target=controller.app.run, kwargs={'host': '0.0.0.0', 'port': 5000, 'debug': False, 'threaded': True})
    ros_thread = threading.Thread(target=rclpy.spin, args=(controller.node,))

    try:
        # Start both threads
        flask_thread.start()
        ros_thread.start()

        # Wait for threads to finish
        flask_thread.join()
        ros_thread.join()

    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
