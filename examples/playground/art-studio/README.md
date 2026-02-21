# 🎨 Robotic Art Studio

Emotion-responsive painting robots. Abstract art created from real-time sentiment analysis.

## Concept

A fleet of painting robots creates unique abstract artwork based on:
- Twitter/X sentiment feed
- Live music analysis
- Audience mood via webcam
- Random emotional "weather"

Each emotion maps to color, stroke pattern, and movement speed.

## Architecture

```
Emotion Input → Analysis → Bridge → Painting Robots → Canvas
```

## Files

- `art_brain.py` - Main orchestrator
- `emotion_engine.py` - Sentiment → art parameters
- `painter_robot.py` - Simulated painting robot
- `canvas_server.py` - Web display of artwork

## Run

```bash
docker-compose up
# Open http://localhost:8080 to see live painting
```

## Emotion → Art Mapping

| Emotion | Color | Stroke | Speed |
|---------|-------|--------|-------|
| Joy | Yellow/Orange | Circular | Fast |
| Sadness | Blue/Grey | Dripping | Slow |
| Anger | Red/Black | Jagged | Aggressive |
| Calm | Green/Pastel | Flowing | Gentle |
| Wonder | Purple/Gold | Swirling | Medium |