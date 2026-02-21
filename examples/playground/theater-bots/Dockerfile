FROM python:3.11-slim

WORKDIR /app

RUN pip install --no-cache-dir agent-ros-bridge>=0.3.3

COPY *.py ./
COPY *.html ./

EXPOSE 8080

ENV PYTHONUNBUFFERED=1

CMD ["python", "theater_director.py"]