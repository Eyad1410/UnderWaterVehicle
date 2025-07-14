import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import time
from cv_bridge import CvBridge

class Image_Stitching():
    def __init__(self):
        self.ratio = 0.85
        self.min_match = 10
        self.sift = cv2.SIFT_create()
        self.smoothing_window_size = 800

    def registration(self, img1, img2):
        kp1, des1 = self.sift.detectAndCompute(img1, None)
        kp2, des2 = self.sift.detectAndCompute(img2, None)
        if des1 is None or des2 is None:
            print("❌ No descriptors found!")
            return None
        matcher = cv2.BFMatcher()
        raw_matches = matcher.knnMatch(des1, des2, k=2)
        good_points, good_matches = [], []
        for m1, m2 in raw_matches:
            if m1.distance < self.ratio * m2.distance:
                good_points.append((m1.trainIdx, m1.queryIdx))
                good_matches.append([m1])
        if len(good_points) > self.min_match:
            image1_kp = np.float32([kp1[i].pt for (_, i) in good_points])
            image2_kp = np.float32([kp2[i].pt for (i, _) in good_points])
            H, status = cv2.findHomography(image2_kp, image1_kp, cv2.RANSAC, 5.0)
            return H
        print("❌ Not enough good matches.")
        return None

    def create_mask(self, img1, img2, version):
        h1, w1 = img1.shape[:2]
        w2 = img2.shape[1]
        h_panorama = h1
        w_panorama = w1 + w2
        offset = int(self.smoothing_window_size / 2)
        barrier = w1 - offset
        mask = np.zeros((h_panorama, w_panorama))
        if version == 'left_image':
            mask[:, barrier - offset:barrier + offset] = np.tile(np.linspace(1, 0, 2 * offset).T, (h_panorama, 1))
            mask[:, :barrier - offset] = 1
        else:
            mask[:, barrier - offset:barrier + offset] = np.tile(np.linspace(0, 1, 2 * offset).T, (h_panorama, 1))
            mask[:, barrier + offset:] = 1
        return cv2.merge([mask, mask, mask])

    def blending(self, img1, img2):
        H = self.registration(img1, img2)
        if H is None:
            print("❌ Failed to find homography. Skipping this pair.")
            return img1
        h1, w1 = img1.shape[:2]
        w2 = img2.shape[1]
        h_panorama = h1
        w_panorama = w1 + w2

        panorama1 = np.zeros((h_panorama, w_panorama, 3), dtype=np.float32)
        mask1 = self.create_mask(img1, img2, version='left_image')
        panorama1[0:h1, 0:w1, :] = img1.astype(np.float32)
        panorama1 *= mask1

        mask2 = self.create_mask(img1, img2, version='right_image')
        panorama2 = cv2.warpPerspective(img2, H, (w_panorama, h_panorama)).astype(np.float32) * mask2

        result = panorama1 + panorama2

        rows, cols = np.where(result[:, :, 0] > 1)
        if len(rows) == 0 or len(cols) == 0:
            print("❌ Cropping failed: No nonzero pixels.")
            return None
        min_row, max_row = min(rows), max(rows) + 1
        min_col, max_col = min(cols), max(cols) + 1
        final_result = result[min_row:max_row, min_col:max_col, :]
        final_result = np.clip(final_result, 0, 255).astype(np.uint8)
        return final_result

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')
        self.bridge = CvBridge()
        self.capturing_images = False
        self.last_capture_time = 0
        self.image_directory = '/path/to/images'  # Change as needed
        self.image_buffer = []
        self.image_count = 0
        self.logger = self.get_logger()
        self.subscription = self.create_subscription(
            Image,
            'your/image/topic',
            self.image_callback,
            10)
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)

    def image_callback(self, msg):
        if self.capturing_images:
            current_time = time.time()
            if current_time - self.last_capture_time >= 0.25:
                self.last_capture_time = current_time
                try:
                    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    image_path = os.path.join(self.image_directory, f"image_{self.image_count:04d}.jpg")
                    cv2.imwrite(image_path, image)
                    self.logger.info(f"✅ Saved image: {image_path}")
                    self.image_buffer.append(image)

                    # --- STITCH every 2 images ---
                    if len(self.image_buffer) == 2:
                        img1, img2 = self.image_buffer
                        stitcher = Image_Stitching()
                        pano = stitcher.blending(img1, img2)
                        if pano is not None:
                            pano_path = os.path.join(self.image_directory, f"panorama_{self.image_count:04d}.jpg")
                            cv2.imwrite(pano_path, pano)
                            self.logger.info(f"🧵 Panorama saved: {pano_path}")
                        else:
                            self.logger.warn("❌ Stitching failed: Not enough features or overlap?")
                        self.image_buffer = []

                    self.image_count += 1

                except Exception as e:
                    self.logger.error(f"❌ Failed to save or stitch image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    node.capturing_images = True  # Set to True to start capturing
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



