from flask import Flask, request, jsonify, send_from_directory, render_template_string
from flask_cors import CORS
import face_recognition
import pickle
import numpy as np
import cv2
import base64
import io
import os

app = Flask(__name__)
CORS(app)

# Load known faces and embeddings once at startup
model_path = "encodings.pickle"
with open(model_path, "rb") as f:
    data = pickle.load(f)

# Threshold for face distance to mark recognized faces
THRESHOLD = 0.4

def read_image_file(file_stream):
    file_bytes = np.frombuffer(file_stream.read(), np.uint8)
    image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    return image

def read_image_base64(base64_str):
    img_data = base64.b64decode(base64_str)
    np_arr = np.frombuffer(img_data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image

@app.route("/recognize-face", methods=["POST"])
def recognize_face():

    if "image" in request.files:
        image_file = request.files["image"]
        image = read_image_file(image_file)
    else:
        data_json = request.get_json()
        if data_json is None or "image_base64" not in data_json:
            return jsonify({"error": "No image file or base64 image provided"}), 400
        image = read_image_base64(data_json["image_base64"])

    if image is None:
        return jsonify({"error": "Invalid image data"}), 400

    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Detect faces
    boxes = face_recognition.face_locations(rgb_image, model="hog")
    encodings = face_recognition.face_encodings(rgb_image, boxes)

    faces_output = []

    for (top, right, bottom, left), encoding in zip(boxes, encodings):
        face_distances = face_recognition.face_distance(data["encodings"], encoding)

        name = "Unknown"
        similarity = None
        if len(face_distances) > 0:
            best_match_index = face_distances.argmin()
            best_distance = face_distances[best_match_index]
            if best_distance <= THRESHOLD:
                name = data["names"][best_match_index]
                similarity = round(float(best_distance), 4)

        faces_output.append({
            "name": name,
            "similarity": similarity,
            "bounding_box": {"top": top, "right": right, "bottom": bottom, "left": left}
        })

        # Draw bounding box and name on the image
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(image, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    response = {
        "num_faces": len(faces_output),
        "faces": faces_output
    }

    return jsonify(response)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=int(os.environ.get('PORT', 5000)))
