import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point

# Initialize ROS node
rospy.init_node('object_tracking_node')

# Load YOLO model
model_path = 'best.pt'
model = YOLO(model_path)
threshold = 0.5

# Initialize the bridge between ROS and OpenCV
bridge = CvBridge()

# Define the ROS publisher
pub = rospy.Publisher('/object_position', Point, queue_size=10)

# Function to publish 3D position
def publish_3d_position(x, y, z):
    # Create a Point message
    point_msg = Point()
    point_msg.x = x
    point_msg.y = y
    point_msg.z = z

    # Publish the message
    pub.publish(point_msg)

# Callback for synchronized messages
def callback(image_msg, depth_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    except CvBridgeError as e:
        print(e)

    results = model(cv_image)[0]
    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        if score > threshold:
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Draw bounding box on the image
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(cv_image, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            # Get Z coordinate from depth image
            z = depth_image[center_y, center_x]

            # Publish 3D position
            publish_3d_position(center_x, center_y, z)

    # Display the image
    cv2.imshow("Detection", cv_image)
    cv2.waitKey(1)


# Subscribe to image and depth topics using message_filters
image_sub = message_filters.Subscriber('/camera/image_raw', Image)
depth_sub = message_filters.Subscriber('/camera/depth', Image)

ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
ts.registerCallback(callback)

rospy.spin()

