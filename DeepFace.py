import cv2
from deepface import DeepFace
import mediapipe as mp
from concurrent.futures import ThreadPoolExecutor

# Initialize Mediapipe components
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
mp_face_mesh = mp.solutions.face_mesh

# Define custom drawing specs
landmark_drawing_spec = mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)
connection_drawing_spec = mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=1)

executor = ThreadPoolExecutor(max_workers=2)  # Initialize ThreadPoolExecutor

def save_user(face_roi, user_image_path="user_image.jpg"):
    """Saves the detected user's face."""
    cv2.imwrite(user_image_path, face_roi)
    print("[INFO] User face saved as the default user.")
    return user_image_path

def analyze_user(image_path):
    """Analyzes the user's face and returns the embedding."""
    try:
        analysis = DeepFace.represent(img_path=image_path, model_name="VGG-Face")
        print("[INFO] User analysis complete.")
        return analysis[0]["embedding"]
    except Exception as e:
        print(f"[ERROR] Failed to analyze user: {e}")
        return None

def analyze_user_async(image_path):
    """Asynchronously analyzes the user's face."""
    return executor.submit(analyze_user, image_path)

def search_user(face_roi, user_image_path):
    """
    Compares the detected face with the saved user's embedding,
    calculates a confidence score, and returns the full result.
    """
    try:
        cv2.imwrite("temp_face.jpg", face_roi)
        result = DeepFace.verify(img1_path="temp_face.jpg", img2_path=user_image_path, model_name="SFace")
        # Calculate a confidence score using the returned distance and threshold
        threshold = result.get("threshold", None)
        distance = result.get("distance", None)
        if threshold is not None and distance is not None and threshold > 0:
            # Normalize the confidence to a 0-1 scale
            confidence = max(0, min(1, (threshold - distance) / threshold))
        else:
            confidence = 0.0
        result["confidence"] = confidence
        return result  # Contains 'verified' and 'confidence'
    except Exception as e:
        print(f"[ERROR] Search failed: {e}")
        return {"verified": False, "confidence": 0.0}

def search_user_async(face_roi, user_image_path):
    """Asynchronously searches for the user."""
    return executor.submit(search_user, face_roi, user_image_path)

def main():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set to 640x480 or lower
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("[ERROR] Cannot access webcam.")
        return

    print("Webcam opened. Press 'q' to quit, 'a' for analysis mode, 's' for search mode.")
    mode = "idle"
    user_image_path = "user_image.jpg"
    user_data = None

    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic, \
         mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:
        
        frame_counter = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Failed to read frame.")
                break

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))

            frame_counter += 1

            if frame_counter % 2 == 0:  # Process every 2nd frame
                for (x, y, w, h) in detections:
                    face_roi = frame[y:y + h, x:x + w]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    if mode == "analysis":
                        user_image_path = save_user(face_roi)
                        future = analyze_user_async(user_image_path)
                        user_data = future.result()  # Wait for analysis to complete
                        mode = "idle"

                    elif mode == "search":
                        if user_data is None:
                            print("[WARNING] No user data available. Please run analysis mode first.")
                            mode = "idle"
                        else:
                            future = search_user_async(face_roi, user_image_path)
                            verification_result = future.result()  # Wait for search to complete

                            if verification_result["verified"]:
                                confidence_percentage = verification_result["confidence"] * 100
                                cv2.putText(frame, f"User Recognized ({confidence_percentage:.1f}%)", (x, y - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                                # Process Mediapipe holistic landmarks
                                image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                holistic_results = holistic.process(image_rgb)
                                face_results = face_mesh.process(image_rgb)

                                # Check if face landmarks are detected
                                if face_results.multi_face_landmarks:
                                    for landmarks in face_results.multi_face_landmarks:
                                        nose_landmark = landmarks.landmark[1]  # Index 1 is typically the nose tip
                                        nose_x = int(nose_landmark.x * frame.shape[1])
                                        nose_y = int(nose_landmark.y * frame.shape[0])
                                        nose_z = nose_landmark.z  # Depth (negative value for distance from camera)
                                        print(f'Nose coordinates: x={nose_x}, y={nose_y}, z={nose_z}')

                                        # Draw face landmarks on the frame
                                        mp_drawing.draw_landmarks(
                                            frame, landmarks, mp_face_mesh.FACEMESH_TESSELATION,
                                            landmark_drawing_spec=landmark_drawing_spec,
                                            connection_drawing_spec=connection_drawing_spec
                                        )

                                # Draw holistic landmarks on the original frame
                                if holistic_results.pose_landmarks:
                                    mp_drawing.draw_landmarks(
                                        frame, holistic_results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                                        landmark_drawing_spec=landmark_drawing_spec,
                                        connection_drawing_spec=connection_drawing_spec
                                    )

                                if holistic_results.right_hand_landmarks:
                                    mp_drawing.draw_landmarks(
                                        frame, holistic_results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                        landmark_drawing_spec=landmark_drawing_spec,
                                        connection_drawing_spec=connection_drawing_spec
                                    )

                                if holistic_results.left_hand_landmarks:
                                    mp_drawing.draw_landmarks(
                                        frame, holistic_results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                        landmark_drawing_spec=landmark_drawing_spec,
                                        connection_drawing_spec=connection_drawing_spec
                                    )
                            else:
                                cv2.putText(frame, "Searching...", (x, y - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                cv2.imshow("Webcam Feed", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[INFO] Exiting program.")
                    break
                elif key == ord('a'):
                    mode = "analysis"
                elif key == ord('s'):
                    mode = "search"

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
