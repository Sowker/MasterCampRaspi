import cv2
import numpy as np
from picamera2 import Picamera2

def init_camera():
    # Initialize Picamera2
    picam = Picamera2()
    picam.configure(picam.create_preview_configuration())
    picam.start()

    try:
        # capture_array()
        prev_frame = picam.capture_array()
        return picam
    except Exception as e:
        picam.stop()
        raise RuntimeError(f"Could not read first frame: {e}")

def shutdown(camera : Picamera2):
    camera.stop()

def analyze_arrow_contours(approx_polygon):
    """
    Analyse géométrique des sommets pour déterminer l'orientation de la flèche.
    Calcule le centre de masse et trouve le sommet le plus excentré (la pointe).
    """
    points = [tuple(pt[0]) for pt in approx_polygon]

    all_x = [p[0] for p in points]
    all_y = [p[1] for p in points]

    center_x = int(float(np.mean(all_x)))
    center_y = int(float(np.mean(all_y)))
    center = (center_x, center_y)

    min_x_pt = min(points, key=lambda p: p[0])  # Le point le plus à gauche de la flèche
    max_x_pt = max(points, key=lambda p: p[0])  # Le point le plus à droite de la flèche

    dist_gauche = abs(min_x_pt[0] - center_x)
    dist_droite = abs(max_x_pt[0] - center_x)

    if dist_gauche > dist_droite:
        pointe = (int(min_x_pt[0]), int(min_x_pt[1]))
        direction = "right"
    else:
        pointe = (int(max_x_pt[0]), int(max_x_pt[1]))
        direction = "left"

    # On retourne la décision, les coordonnées de la pointe et celles du centre
    return direction, pointe, center


def get_direction(picam : Picamera2):
    # capture_array()
    prev_frame = picam.capture_array()

    # Get height and width of the camera feed
    h, w = prev_frame.shape[:2]

    # Define range for white color in HSV
    # (Hue, Saturation, Value)
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([179, 70, 255])

    # Define range for black color in HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 50])

    while True:
        try:
            default_frame = picam.capture_array()
        except RuntimeError as e:
            print(f"Failed to capture frame in loop: {e}")
            break

        # Appy Gaussian Blur
        default_frame = cv2.GaussianBlur(default_frame, (15, 15), 0)

        # ==== To Fine tune ====
        # Crop the image by 10% - on each side - to focus on the center
        frame = default_frame[round(h*0.1):round(h-h*0.11),round(w*0.1):round(w-w*0.1)]
        # frame = default_frame

        # ==== To Fine tune ====
        # Detect the white (out-layer of the arrow) to crop the image to it.
        # So we can remove unnecessary noise

        # Convert the frame from RGB to HSV format
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a white mask to focus on what is inside the white part
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Filter the white part
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Create the matrice of white from applying the mask
        has_white_x = np.any(mask, axis=0)
        has_white_y = np.any(mask, axis=1)

        # Separate each axcis
        x_coords = np.where(has_white_x)[0]
        y_coords = np.where(has_white_y)[0]

        x_min: int
        y_min: int
        x_max: int
        y_max: int

        # Verify that the matrice x and y hold white because we want to detect them
        if not(x_coords.size > 0 and y_coords.size > 0):
            print("No white detected on the image.")
            # cv2.imshow('Raw image', default_frame)

        elif x_coords.size > 0 and y_coords.size > 0:
            x_min, x_max = x_coords[0], x_coords[-1]
            y_min, y_max = y_coords[0], y_coords[-1]

            # print(f"White detected: x({x_min}-{x_max}), y({y_min}-{y_max})")
            box_width = x_max - x_min
            box_height = y_max - y_min
            if box_width < 10 or box_height < 10:
                print(f"Skipping tiny/invalid frame: x({x_min}-{x_max}), y({y_min}-{y_max})")

            else :

                # Draw the rectangle box using the min/max coordinates
                default = cv2.rectangle(default_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Cut the frame to only have the white
                frame = frame[y_min:y_max, x_min:x_max]

                """
                    Boucle principale : capture les images, isole la flèche noire par seuillage binaire,
                    sélectionne le meilleur contour par score géométrique et affiche les résultats.
                    """
                SEUIL_NOIR = 80

                output_frame = frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                _, black_mask = cv2.threshold(blurred, SEUIL_NOIR, 255, cv2.THRESH_BINARY_INV)

                # Éliminer les bruits de pixels blancs isolés et lisser les bordures
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

                contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Variables de mémorisation pour élire la flèche "championne"
                best_contour = None
                best_approx = None
                max_score = -1.0

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 400:

                        # Calcul du périmètre du contour
                        peri = cv2.arcLength(cnt, True)

                        # Approximation polygonale
                        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
                        num_vertices = len(approx)

                        # bonus/pénalités
                        shape_multiplier = 1.0
                        if num_vertices == 7:
                            shape_multiplier = 3.0  # La forme possède exactement les 7 sommets d'une flèche
                        elif num_vertices == 6 or num_vertices == 8:
                            shape_multiplier = 1.5  # La flèche est légèrement déformée par la perspective
                        elif num_vertices < 5 or num_vertices > 10:
                            shape_multiplier = 0.1  # La forme n'a aucun rapport avec un polygone de flèche

                        score = area * shape_multiplier

                        if score > max_score:
                            max_score = score
                            best_contour = cnt
                            best_approx = approx

                if best_contour is not None and best_approx is not None:
                    cv2.drawContours(output_frame, [best_contour], -1, (0, 255, 0), 3)
                    direction, pointe, center = analyze_arrow_contours(best_approx)
                    return direction


def adjust_position(picam: Picamera2):
    # Get height and width of the camera feed from an initial check
    prev_frame = picam.capture_array()
    h, w = prev_frame.shape[:2]

    lower_white = np.array([0, 0, 150])
    upper_white = np.array([179, 70, 255])

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 50])

    while True:
        try:
            default_frame = picam.capture_array()
        except RuntimeError as e:
            print(f"Failed to capture frame in loop: {e}")
            break

        # Apply Gaussian Blur
        blurred_frame = cv2.GaussianBlur(default_frame, (15, 15), 0)

        # Convert the full frame from BGR to HSV format
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        # Create a white mask to find the boundaries
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Create the matrix of white from applying the mask
        has_white_x = np.any(white_mask, axis=0)
        has_white_y = np.any(white_mask, axis=1)

        # Separate each axis
        x_coords = np.where(has_white_x)[0]
        y_coords = np.where(has_white_y)[0]

        # Verify that we actually detected white
        if not (x_coords.size > 0 and y_coords.size > 0):
            print("No white detected on the image.")
            continue

        x_min, x_max = x_coords[0], x_coords[-1]
        y_min, y_max = y_coords[0], y_coords[-1]

        box_width = x_max - x_min
        box_height = y_max - y_min

        if box_width < 10 or box_height < 10:
            print(f"Skipping tiny/invalid frame: x({x_min}-{x_max}), y({y_min}-{y_max})")
            continue

        # Isolate the white bounding box area from our HSV image safely
        cropped_hsv = hsv[y_min:y_max, x_min:x_max]

        # Create mask for the black color inside this cropped space
        black_mask = cv2.inRange(cropped_hsv, lower_black, upper_black)

        # Calculate the moments of the binary black mask
        moments = cv2.moments(black_mask)

        # Prevent division by zero
        if moments["m00"] != 0:
            # Calculate relative X and Y coordinates of the center (within the crop)
            relative_cX = int(moments["m10"] / moments["m00"])
            relative_cY = int(moments["m01"] / moments["m00"])

            # Translate to ABSOLUTE coordinates (relative to the full screen) using the offsets
            cX = x_min + relative_cX
            cY = y_min + relative_cY

            # Screen absolute middle alignment logic
            x_screen = w / 2
            tolerance = 30

            if cX > (x_screen + tolerance):
                return "right"
            elif cX < (x_screen - tolerance):
                return "left"
            else:
                return "straight"