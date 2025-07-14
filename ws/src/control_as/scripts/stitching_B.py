import cv2
import os

# Paths
IMAGE_FOLDER = "/UnderWaterVehicle/ws/src/control_as/images"
OUTPUT_FOLDER = "/UnderWaterVehicle/ws/src/control_as/scripts/final_super_pairs"
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# Multi-batch super stitch input groups
batch_groups = [
    (
        ["image_0026.jpg", "image_0032.jpg"],
        ["image_0027.jpg", "image_0033.jpg"],
        ["image_0028.jpg", "image_0034.jpg"],
        ["image_0029.jpg", "image_0035.jpg"]
    ),
    (
        ["image_0038.jpg", "image_0039.jpg"],
        ["image_0040.jpg", "image_0041.jpg"],
        ["image_0042.jpg", "image_0048.jpg"],
        ["image_0043.jpg", "image_0049.jpg"]
    ),
    (
        ["image_0056.jpg", "image_0058.jpg"],
        ["image_0057.jpg", "image_0059.jpg"],
        ["image_0060.jpg", "image_0061.jpg"],
        ["image_0062.jpg", "image_0063.jpg"],
        ["image_0064.jpg"]
    )    
]

def crop_black_borders(pano):
    gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return pano
    x, y, w, h = cv2.boundingRect(contours[0])
    return pano[y:y+h, x:x+w]

def try_stitch_images(image_paths, output_path):
    images = []
    for name in image_paths:
        full_path = os.path.join(IMAGE_FOLDER, name)
        img = cv2.imread(full_path)
        if img is None:
            print(f"❌ Could not read image: {name}")
            return False
        images.append(img)

    try:
        stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
    except AttributeError:
        stitcher = cv2.createStitcher(False)  # For older OpenCV versions

    stitcher.setWaveCorrection(False)

    status, pano = stitcher.stitch(images)
    if status == cv2.Stitcher_OK:
        pano_cropped = crop_black_borders(pano)
        cv2.imwrite(output_path, pano_cropped)
        print(f"✅ Successfully stitched: {image_paths} → {output_path}")
        return True
    else:
        print(f"❌ Failed to stitch: {image_paths} (OpenCV status: {status})")
        return False

# Process each group
for idx, group in enumerate(batch_groups, 1):
    combined = [img for batch in group for img in batch]
    output_file = os.path.join(OUTPUT_FOLDER, f"superbatch_{idx:02d}.jpg")
    try_stitch_images(combined, output_file)

