#!/usr/bin/env python2
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import threading

class ImageServer:
    def __init__(self, port=51121):
        rospy.init_node('image_server', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.update_time = time.time()
        self.lock = threading.Lock()
        
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', port))
        self.server.listen(5)
        print("Server starting on 0.0.0.0:{}".format(port))
        
        threading.Thread(target=self.handle_requests).start()
        rospy.spin()
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_image = cv_image
                self.update_time = time.time()
        except:
            pass
    
    def handle_requests(self):
        while not rospy.is_shutdown():
            try:
                conn, addr = self.server.accept()
                threading.Thread(target=self.handle_client, args=(conn,)).start()
            except:
                break
    
    def handle_client(self, conn):
        try:
            request = conn.recv(1024)
            if b'GET /image' in request:
                print("{}: GET /image (update_time={})".format(time.time(), self.update_time))
                with self.lock:
                    if self.latest_image is not None:
                        import cv2
                        _, buffer = cv2.imencode('.jpg', self.latest_image)
                        jpg_data = buffer.tobytes()
                        
                        response = (
                            "HTTP/1.1 200 OK\r\n"
                            "Content-Type: image/jpeg\r\n"
                            "Content-Length: %d\r\n"
                            "Access-Control-Allow-Origin: *\r\n"
                            "\r\n"
                        ) % len(jpg_data)
                        
                        conn.send(response.encode())
                        conn.send(jpg_data)
                    else:
                        conn.send(b"HTTP/1.1 503 No Image\r\n\r\n")
            else:
                conn.send(b"HTTP/1.1 404 Not Found\r\n\r\n")
        except:
            pass
        finally:
            conn.close()

if __name__ == '__main__':
    ImageServer(51121)
