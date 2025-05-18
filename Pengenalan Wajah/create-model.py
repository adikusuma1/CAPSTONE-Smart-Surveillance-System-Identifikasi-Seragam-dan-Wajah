import os
import face_recognition
import cv2
import pickle

def main(dataset_dir, model_output_path):
    """
    Create a face recognition model from dataset directory.
    Dataset structure: dataset_dir/person_name/imagefiles.jpg
    Saves encoded face embeddings and names into a pickle file.
    """
    print("[INFO] Quantifying faces...")
    known_encodings = []
    known_names = []

    # Loop over each person in dataset
    for person_name in os.listdir(dataset_dir):
        person_dir = os.path.join(dataset_dir, person_name)
        if not os.path.isdir(person_dir):
            continue
        print(f"[INFO] Processing folder for: {person_name}")

        # Loop over images of the person
        for image_name in os.listdir(person_dir):
            image_path = os.path.join(person_dir, image_name)
            image = cv2.imread(image_path)
            if image is None:
                print(f"[WARNING] Unable to read image: {image_path}")
                continue
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Detect faces
            boxes = face_recognition.face_locations(rgb, model="hog")
            if len(boxes) == 0:
                print(f"[WARNING] No face found in image: {image_path}")
                continue

            # Encode faces
            encodings = face_recognition.face_encodings(rgb, boxes)

            # We only process first face in the image (assuming one face per image)
            known_encodings.append(encodings[0])
            known_names.append(person_name)

    # Save encodings and names to disk
    print("[INFO] Serializing encodings...")
    data = {"encodings": known_encodings, "names": known_names}
    with open(model_output_path, "wb") as f:
        pickle.dump(data, f)

    print(f"[INFO] Model saved to {model_output_path}")

if __name__ == "__main__":
    # Directly set the dataset path and model output path here
    dataset_dir = "dataset"  # <-- Set your dataset folder here
    model_output_path = "encodings.pickle"  # <-- Set your output encoding file path here

    main(dataset_dir, model_output_path)
