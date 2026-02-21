FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY *.py ./
COPY *.html ./

# Create artworks directory
RUN mkdir -p /app/artworks

# Default environment
ENV PYTHONUNBUFFERED=1
ENV STUDIO_MODE=simulation

EXPOSE 8765 8080 8081

CMD ["python", "art_brain.py"]