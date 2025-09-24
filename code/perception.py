import numpy as np
import cv2

# Apply color threshold to identify navigable terrain
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    color_select[above_thresh] = 1
    return color_select

def color_thresh_sample(img, rgb_thresh_low=(110, 110, 0), rgb_thresh_high=(255, 255, 70)):
    """
    Identify yellow samples in the image.
    Default thresholds work for bright yellow.
    """
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be within the threshold range
    within_thresh = (
        (img[:, :, 0] >= rgb_thresh_low[0]) & (img[:, :, 0] <= rgb_thresh_high[0]) &  # Red
        (img[:, :, 1] >= rgb_thresh_low[1]) & (img[:, :, 1] <= rgb_thresh_high[1]) &  # Green
        (img[:, :, 2] >= rgb_thresh_low[2]) & (img[:, :, 2] <= rgb_thresh_high[2])    # Blue
    )
    color_select[within_thresh] = 1
    return color_select

# Perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))
    return warped, mask

# Convert to rover-centric coords
def rover_coords(binary_img):
    ypos, xpos = binary_img.nonzero()
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float32)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float32)
    return x_pixel, y_pixel

# Rotate pixels
def rotate_pix(xpix, ypix, yaw):
    yaw_rad = np.deg2rad(yaw)
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return xpix_rotated, ypix_rotated

# Translate pixels
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    xpix_translated = xpos + (xpix_rot / scale)
    ypix_translated = ypos + (ypix_rot / scale)
    return xpix_translated, ypix_translated

# Convert rover-centric pixel values to world coordinates
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    return x_pix_world, y_pix_world

# Convert to polar coordinates in rover space
def to_polar_coords(x_pixel, y_pixel):
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Main perception step
def perception_step(Rover):
    # NOTE: define calibration box in source (actual) and destination (desired) coordinates
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140],
                         [301, 140],
                         [200, 96],
                         [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset], 
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                             ])

    # Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)

    # Apply color threshold to identify navigable terrain
    threshed = color_thresh(warped)

    # Update vision image (Rover.vision_image is 3-channel)
    Rover.vision_image[:, :, 2] = threshed * 255
    
    ## Detect and mark rock samples
    rock_threshed = color_thresh_sample(warped)
    Rover.vision_image[:, :, 1] = rock_threshed * 255
    
    # Convert map image pixel values to rover-centric coords for rock samples
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
    
    # Convert rover-centric pixel positions to polar coordinates for rock samples
    rock_dist, rock_angles = to_polar_coords(rock_xpix, rock_ypix)
    if len(rock_angles) > 0:
        Rover.rock_dists = rock_dist
        Rover.rock_angles = rock_angles
    else:
        Rover.rock_dists = None
        Rover.rock_angles = None

    # Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)

    # Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 10
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # Update Rover worldmap (navigable terrain)
    Rover.worldmap[y_world, x_world, 2] += 1

    # Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    Rover.nav_dists = dist
    Rover.nav_angles = angles

    return Rover
