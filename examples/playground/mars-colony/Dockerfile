FROM python:3.11-slim

WORKDIR /app

RUN pip install --no-cache-dir agent-ros-bridge>=0.3.3

COPY *.py ./
COPY *.html ./

ENV PYTHONUNBUFFERED=1

CMD ["python", "mars_command.py"]