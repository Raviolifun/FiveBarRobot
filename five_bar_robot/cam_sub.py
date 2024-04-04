import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, "/blackfly_0/image_raw", self.listener_callback, 10)
        self.publisher = self.create_publisher(msg_type=PointStamped, topic="puck_point", qos_profile=1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        # def nothing(x):
        #    pass

        # Create a window
        # cv2.namedWindow('image')

        # create trackbars for color change
        # cv2.createTrackbar('lowH','image',0,179,nothing)
        # cv2.createTrackbar('highH','image',179,179,nothing)

        # cv2.createTrackbar('lowS','image',0,255,nothing)
        # cv2.createTrackbar('highS','image',255,255,nothing)

        # cv2.createTrackbar('lowV','image',0,255,nothing)
        # cv2.createTrackbar('highV','image',255,255,nothing)

    def listener_callback(self, data):
        """
        Callback function.
        """

        def plotCircles(img, circles, color):
            if circles is None:
                return

            for (x, y, r) in circles[0]:
                cv2.circle(img, (int(x), int(y)), int(r), color, 2)

        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        drawImg = current_frame

        # resize image (half-size) for easier processing
        resized = cv2.resize(current_frame, None, fx=0.5, fy=0.5)
        frame = resized

        # convert to single-channel image
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # while(True):
        # frame = current_frame
        # get current positions of the trackbars
        # ilowH = cv2.getTrackbarPos('lowH', 'image')
        # ihighH = cv2.getTrackbarPos('highH', 'image')
        # ilowS = cv2.getTrackbarPos('lowS', 'image')
        # ihighS = cv2.getTrackbarPos('highS', 'image')
        # ilowV = cv2.getTrackbarPos('lowV', 'image')
        # ihighV = cv2.getTrackbarPos('highV', 'image')

        # convert color to hsv because it is easy to track colors in this color model
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lower_hsv = numpy.array([ilowH, ilowS, ilowV])
        # higher_hsv = numpy.array([ihighH, ihighS, ihighV])
        # Apply the cv2.inrange method to create a mask
        # mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
        # Apply the mask on the image to extract the original color
        # flt = cv2.bitwise_and(frame, frame, mask=mask)
        # cv2.imshow('image', flt)
        # cv2.waitKey(1)
        # Press q to exit
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break

        # threshold grayscale to binary (black & white) image
        threshVal = 50
        ret, thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
        # drawhsv = cv2.cvtColor(hsv, cv2.COLOR_GRAY2BGR)

        # Find contours
        # items = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # contours = items[0] if len(items) == 2 else items[1]
        # cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=5, lineType=cv2.LINE_AA)
        # try:
        # drawImg = cv2.cvtColor(thr, cv2.COLOR_GRAY2BGR)
        # Get centroid from moments - may not really need this - could get from bounding box
        # M = cv2.moments(contours[0])
        # cX = int(M["m10"] / M["m00"])
        # cY = int(M["m01"] / M["m00"])
        # print(f'Centroid: cX={cX}, cY={cY} do not forget 400 offset of ROI')
        # Mark centre in black
        # thr[cY-3:cY+3, cX-3:cX+3] = 0
        # Get bounding box and draw it on
        # x, y, w, h = cv2.boundingRect(contours[0])
        # print(f'x={x}, y={y}, w={w}, h={h}')
        # print(f'cX={x+w/2}, cY={y+h/2}')
        # cv2.rectangle(drawImg, (x, y), (x + w, y + h), 255, 1)
        # cv2.imshow("inv", drawImg)

        # cv2.waitKey(1)
        # except:
        # pass
        # detect outer pump circle
        msg_out = PointStamped()
        Circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 10000, param2=9, minRadius=6, maxRadius=10)
        if Circles is not None:
            c = Circles[0, :].astype("int")
            xp = (c[0, 0] - 40) * 0.19 * 0.0254  # meter
            yp = (c[0, 1] - 265) * -0.19 * 0.0254  # meter

            msg_out.header.frame_id = "robot"
            msg_out.header.stamp = self.get_clock().now().to_msg()

            msg_out.point.x = xp
            msg_out.point.y = yp
            self.publisher.publish(msg_out)
            # plotCircles(drawImg, Circles, (0, 255, 0))

        # if (pumpCircles is None):
        #    raise Exception("No pump circles found!")
        # elif len(pumpCircles[0])!=1:
        #    raise Exception("Wrong # of pump circles: found {} expected {}".format(len(pumpCircles[0]),1))
        # else:
        #    pumpCircle = pumpCircles[0][0]

        # Display image
        cv2.imshow("Hough", drawImg)
        # cv2.imshow("HSV", mask)
        # cv2.imshow("Contours", frame)

        cv2.waitKey(1)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
