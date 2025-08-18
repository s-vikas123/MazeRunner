#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO


class YoloClassifierNode(Node):
    def __init__(self):
        super().__init__('yolo_classifier_node')

        self.bridge = CvBridge()
        self.model = YOLO('/home/keerthi/ros2_ws/src/lab6/lab6_final/last.pt')

        # Subscribe to compressed image topic
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/simulated_camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # Publish detected class
        self.class_publisher = self.create_publisher(
            String,
            'detected_class',
            10
        )

        self.get_logger().info('YOLO Classifier Node with Camera Input Initialized')

    def image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Show original image
            cv2.imshow("Original Camera View", cv_image)

            # Pre-process the image
            processed_image = self.preprocess_image(cv_image)

            # Show processed image
            cv2.imshow("Processed Camera View", processed_image)
            cv2.waitKey(1)

            # Run YOLO classification on both images
            results_raw = self.model(cv_image)
            results_processed = self.model(processed_image)

            # Extract probabilities
            probs_raw = results_raw[0].probs.data.cpu().numpy().tolist()
            probs_processed = results_processed[0].probs.data.cpu().numpy().tolist()

            # Get top classes and scores
            top_raw_index = int(np.argmax(probs_raw))
            top_raw_score = probs_raw[top_raw_index]

            top_processed_index = int(np.argmax(probs_processed))
            top_processed_score = probs_processed[top_processed_index]

            # Choose the one with higher confidence
            if top_processed_score >= top_raw_score:
                selected_class = top_processed_index
            else:
                selected_class = top_raw_index

            # Publish the selected class index
            class_msg = String()
            class_msg.data = str(selected_class)
            self.class_publisher.publish(class_msg)

            self.get_logger().info(f'class: {selected_class}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

   


    def preprocess_image(self, img):
        """
        Preprocesses the image with various steps like brightness/contrast adjustment, gamma correction, etc.
        """

        # # Step 1: Adjust brightness and contrast
        # img = self.adjust_brightness_contrast(img)

        # # Step 2: Apply gamma correction (if needed)
        # img = self.gamma_correction(img)

        # # Step 3: Sharpen the image
        # img = self.sharpen_image(img)

        # # Step 4: Optional: Denoise the image
        # img = self.denoise_image(img)

        img = self.crop_img(img)

        # Step 5: Optional: Upscale the image (for higher resolution)
        img = self.upscale_image(img)

        # Step 6: Optional: Edge detection (if required for feature extraction)
        # img = self.detect_edges(img)

        return img

    def adjust_brightness_contrast(self, img, alpha=1.5, beta=50):
        """
        Adjusts the brightness and contrast of the image.
        alpha = contrast control
        beta = brightness control
        """
        return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    def gamma_correction(self, img, gamma=1.2):
        """
        Applies gamma correction to the image.
        """
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(img, table)

    def sharpen_image(self, img):
        """
        Sharpens the image using a kernel.
        """
        kernel = np.array([[0, -1, 0], [-1, 5,-1], [0, -1, 0]])
        return cv2.filter2D(img, -1, kernel)

    def denoise_image(self, img):
        """
        Denoises the image using Gaussian Blur.
        """
        return cv2.GaussianBlur(img, (5, 5), 0)

    def upscale_image(self, img, scale=3):
        """
        Upscales the image by a factor of 'scale' (default is 2).
        """
        height, width = img.shape[:2]
        new_dim = (width * scale, height * scale)
        return cv2.resize(img, new_dim, interpolation=cv2.INTER_LINEAR)
    
    def crop_img(self, img, scale=2.0):
        h, w = img.shape[:2]
    
        # Calculate the zoomed size based on the scale factor
        zoom = 1 / scale
        crop_h, crop_w = int(h * zoom), int(w * zoom)

        # Calculate the starting point for cropping (center cropping)
        start_h, start_w = (h - crop_h) // 2, (w - crop_w) // 2

        # Crop the image
        cropped_img = img[start_h:start_h + crop_h, start_w:start_w + crop_w]

        # Resize cropped image to 64x64 pixels
        resized_img = cv2.resize(cropped_img, (64, 64), interpolation=cv2.INTER_LINEAR)

        return resized_img


def main(args=None):
    rclpy.init(args=args)
    node = YoloClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()