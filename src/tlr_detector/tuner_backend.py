import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, Response, send_from_directory
import numpy as np
import logging
import time

# --- Global variables ---
g_cv_image = None
g_lock = threading.Lock()
g_bridge = CvBridge()

# --- ROS2 Subscriber ---
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('hsv_tuner_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/traffic_light_image',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribing to /traffic_light_image...')

    def listener_callback(self, msg):
        global g_cv_image, g_lock
        try:
            cv_image = g_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with g_lock:
                g_cv_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def run_ros_node():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

# --- Flask Web Server ---
app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Suppress Flask's default logging

@app.route('/')
def index():
    return send_from_directory('.', 'hsv_tuner.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global g_cv_image, g_lock
        while True:
            frame_to_send = None
            with g_lock:
                if g_cv_image is not None:
                    frame_to_send = g_cv_image.copy()
            
            if frame_to_send is None:
                # If no image, create a placeholder
                frame_to_send = np.zeros((200, 400, 3), dtype=np.uint8)
                cv2.putText(frame_to_send, 'Waiting for ROS topic...', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame_to_send)
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            # Control frame rate
            time.sleep(0.1) 

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting HSV Tuner backend...")
    print("1. Make sure your ROS environment is sourced.")
    print("2. Run your ROS nodes that publish the '/traffic_light_image' topic.")
    print("3. Open your web browser and go to: http://127.0.0.1:5000")
    
    # Run ROS node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5000, debug=False)