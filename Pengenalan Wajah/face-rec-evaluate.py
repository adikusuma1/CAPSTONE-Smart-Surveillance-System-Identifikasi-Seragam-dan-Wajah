import os
import cv2
import pickle
import face_recognition
import numpy as np
from sklearn.metrics import classification_report, confusion_matrix

# Path ke model dan dataset
MODEL_PATH = "encodings.pickle"
DATASET_DIR = "dataset"
THRESHOLD = 0.4  # Jika distance > threshold, maka dianggap Unknown

# Load model
with open(MODEL_PATH, "rb") as f:
    data = pickle.load(f)

y_true = []
y_pred = []

# Proses setiap folder (nama orang)
for person_name in os.listdir(DATASET_DIR):
    person_dir = os.path.join(DATASET_DIR, person_name)
    if not os.path.isdir(person_dir):
        continue

    for filename in os.listdir(person_dir):
        image_path = os.path.join(person_dir, filename)

        image = cv2.imread(image_path)
        if image is None:
            continue

        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        boxes = face_recognition.face_locations(rgb, model="hog")
        encodings = face_recognition.face_encodings(rgb, boxes)

        # Label ground truth: Unknown jika dari folder Random
        label = "Unknown" if person_name.lower() == "random" else person_name

        if len(encodings) == 0:
            y_true.append(label)
            y_pred.append("NoFace")
            continue

        for encoding in encodings:
            distances = face_recognition.face_distance(data["encodings"], encoding)

            name = "Unknown"
            if len(distances) > 0:
                best_match_index = np.argmin(distances)
                best_distance = distances[best_match_index]
                if best_distance <= THRESHOLD:
                    name = data["names"][best_match_index]

            print(f"File: {image_path}, Folder: {person_name}, Prediksi: {name}, Distance: {best_distance}")

            y_true.append(label)
            y_pred.append(name)

# Tampilkan evaluasi
print("Classification Report:")
print(classification_report(y_true, y_pred))

print("Confusion Matrix:")
print(confusion_matrix(y_true, y_pred))

correct = sum(1 for true, pred in zip(y_true, y_pred) if true == pred)
accuracy = correct / len(y_true)
print(f"\nAccuracy: {accuracy:.2f}")
