FROM python:3.10-slim

# Install system dependencies required by dlib and OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libopenblas0 \
    libgl1-mesa-glx \
    liblapack3 \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . /app

RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt

CMD ["python", "face-rec-api.py"]
