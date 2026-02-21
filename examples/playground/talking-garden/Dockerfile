FROM python:3.11-slim

WORKDIR /app

# Install dependencies
RUN pip install --no-cache-dir agent-ros-bridge>=0.3.3

# Copy application
COPY *.py ./
COPY *.html ./
COPY docker-compose.yml ./

# Create poems directory
RUN mkdir -p /app/poems

ENV PYTHONUNBUFFERED=1

CMD ["python", "garden_oracle.py"]