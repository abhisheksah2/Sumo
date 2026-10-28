# Use a lightweight official Python Linux base
FROM python:3.11-slim

# Install system dependencies and the core SUMO engine
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sumo \
    sumo-tools \
    && rm -rf /var/lib/apt/lists/*

# Set the environment variable so TraCI knows where SUMO binaries reside
ENV SUMO_HOME=/usr/share/sumo

# Set working directory inside the container
WORKDIR /app

# Copy requirement list and install Python packages
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy all project files (sumocfg, net.xml, trips, python script)
COPY . .

# Expose the WebSocket port (adjust 3004 if your backend listens on another port)
EXPOSE 3004

# Launch the backend twin script
CMD ["python", "backend_twin.py"]
