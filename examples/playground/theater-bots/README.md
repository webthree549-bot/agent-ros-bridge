# 🎭 Interactive Theater Bots

Robotic actors that perform improvised scenes based on audience input.

## Concept

A troupe of robot performers that:
- Accept audience prompts via chat/WebSocket
- Improvise scenes using generative techniques
- Coordinate movements and "dialogue"
- React to audience reactions

## Performance Types

1. **Shakespeare Bots** - iambic pentameter dialogue
2. **Absurdist Bots** - Beckett-style existential scenes
3. **Action Bots** - High-energy choreographed sequences
4. **Romance Bots** - Melodramatic love stories
5. **Horror Bots** - Atmospheric suspense

## Files

- `theater_director.py` - Scene orchestration
- `actor_bots.py` - Individual performers
- `audience_input.py` - Prompt handling
- `script_generator.py` - Real-time script creation

## Run

```bash
docker-compose up
# Web interface at http://localhost:8080
# Submit prompts, watch performance
```