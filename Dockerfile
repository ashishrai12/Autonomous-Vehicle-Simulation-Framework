# Stage 1: Build Rust Engine
FROM rust:1.75-slim as rust-builder

WORKDIR /app
RUN apt-get update && apt-get install -y python3-dev python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install maturin

COPY rust_engine /app/rust_engine
WORKDIR /app/rust_engine
RUN maturin build --release --out /app/dist

# Stage 2: Final Image
FROM python:3.10-slim

# Install system dependencies for GUI and CUDA
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy built wheels from rust-builder
COPY --from=rust-builder /app/dist/*.whl /app/dist/
RUN pip install /app/dist/*.whl

# Copy Python source code
COPY . /app/
RUN pip install -r requirements.txt || true

# Environment variables for X11 forwarding
ENV DISPLAY=:0

CMD ["python", "src/main.py"]
