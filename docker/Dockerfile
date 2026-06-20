FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    libv4l-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY models/ models/
COPY src/person_detection.py .

ENV DRONE_CAMERA=0
ENV PIXHAWK_PORT=/dev/ttyACM0

EXPOSE 8080

ENTRYPOINT ["python3", "person_detection.py", \
            "--model", "models/person_detector.onnx", \
            "--stream"]
