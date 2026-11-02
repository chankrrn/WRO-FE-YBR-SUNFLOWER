import numpy as np
import cv2
from utils.image_color_utils import ImageColorUtils
from utils.enums import Color

BLUR_FILTER_SIZE = 9            # size of the bilateral filter
BLUR_SIGMA_COLOR = 75           # how much colors can differ to be considered similar
BLUR_SIGMA_SPACE = 75           # how far pixels can be to influence each other
THRESHOLD = 110                 # threshold value for binary image
WHITE_VALUE = 255               # to set white color in binary image
WHITE_COLOR = (255, 255, 255)   # BGR white color for OpenCV
MATRIX_SIZE = 5                 # size of the kernel for morphological operations


class ImageTransformUtils:
    """
    A utility class for various image transformation operations.
    """

    CAMERA_PIC_WIDTH = 640                 # reduced image width
    CAMERA_PIC_HEIGHT = 580                # reduced image height
    # Digital zoom: keep a centred window 1/ZOOM the size (1.0 = no zoom).
    # Cropped only, never resized, so each pixel still covers the same angle
    # and ObjectSolver's DEG_PER_PX stays valid.
    ZOOM = 1.25
    PIC_WIDTH = int(CAMERA_PIC_WIDTH / ZOOM)
    PIC_HEIGHT = int(CAMERA_PIC_HEIGHT / 2 / ZOOM)             # middle half of the frame, zoomed
    PIC_X_START = (CAMERA_PIC_WIDTH - PIC_WIDTH) // 2          # first column kept
    PIC_Y_START = (CAMERA_PIC_HEIGHT - PIC_HEIGHT) // 2        # first row kept

    @staticmethod
    def crop_image(img, x_start, x_end, y_start, y_end):
        return img[y_start:y_end, x_start:x_end]

    @staticmethod
    def bgr_to_hsv(img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    @staticmethod
    def dilate(img):
        kernel = np.ones((3,3), np.uintp)
        dilated_image = cv2.dilate(img, kernel, iterations = 1)
        return dilated_image
    
    @staticmethod
    def erode(img):
        kernel = np.ones((3,3), np.uintp)
        eroded_image = cv2.erode(img, kernel, iterations=2)
        return eroded_image

    @staticmethod
    def remove_color(hsv_img, target_img, color):
        if color == Color.ALL_COLORS:
            maskb = ImageColorUtils.calculate_color_mask(hsv_img, Color.BLUE)
            masko = ImageColorUtils.calculate_color_mask(hsv_img, Color.ORANGE)
            maskg = ImageColorUtils.calculate_color_mask(hsv_img, Color.GREEN)
            maskr = ImageColorUtils.calculate_color_mask(hsv_img, Color.RED)
            mask = cv2.bitwise_or(maskb, masko, maskg, maskr)
            maskp = ImageColorUtils.calculate_color_mask(hsv_img, Color.PINK)
        mask = ImageColorUtils.calculate_color_mask(hsv_img, color)
        target_img[mask > 0] = WHITE_COLOR  # Change color to white
        target_img[maskp > 0] = (0, 0, 0)
        return target_img, mask
    
    @staticmethod
    def exclude_color(hsv_img, color):
        mask = cv2.bitwise_not(ImageColorUtils.calculate_color_mask(hsv_img, color))
        return mask
    
    @staticmethod
    def keep_color(img, color):
        mask = ImageColorUtils.calculate_color_mask(img, color)
             
        #dilated_mask = ImageUtils.dilate(mask)
        #keep_color_img = cv2.bitwise_and(img, img, mask=mask)
        #cv2.imshow("color Mask ", mask)
        return mask

    @staticmethod
    def color_to_grayscale(img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def blur_image(img):
        return cv2.bilateralFilter(img, BLUR_FILTER_SIZE, BLUR_SIGMA_COLOR, BLUR_SIGMA_SPACE)

    @staticmethod
    def make_binary(img):
        _, binary_img = cv2.threshold(img, THRESHOLD, WHITE_VALUE, cv2.THRESH_BINARY)  # Invert threshold
        return binary_img

    @staticmethod
    def clean_binary(img):
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        

