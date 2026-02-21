"""
Painter Robot - Simulated robotic artist
"""
import asyncio
import random


class PaintingRobot:
    """
    A simulated painting robot with unique artistic style.
    Each robot has its own personality and interpretation of emotions.
    """
    
    def __init__(self, name: str, favorite_color: str, style: str):
        self.name = name
        self.favorite_color = favorite_color
        self.style = style
        self.paintings_completed = 0
        self.current_position = {"x": 0, "y": 0}
        self.paint_level = 100.0
        
    async def paint_stroke(self, params: dict) -> dict:
        """
        Execute a painting stroke based on emotional parameters.
        """
        # Simulate painting time based on complexity
        duration = random.uniform(0.5, 2.0) / params.get("speed", 1.0)
        await asyncio.sleep(duration)
        
        # Generate stroke characteristics based on style and emotion
        stroke = self._generate_stroke(params)
        
        # Update state
        self.paintings_completed += 1
        self.paint_level -= random.uniform(0.5, 2.0)
        
        return {
            "robot": self.name,
            "style": self.style,
            "stroke_type": stroke["type"],
            "color": stroke["color"],
            "position": self.current_position.copy(),
            "description": stroke["description"],
            "duration": round(duration, 2)
        }
    
    def _generate_stroke(self, params: dict) -> dict:
        """Generate stroke based on robot style and emotion params"""
        
        # Blend emotion colors with robot's favorite color
        colors = params.get("colors", ["#000000"])
        if random.random() < 0.3:  # 30% chance to use favorite color
            color = self.favorite_color
        else:
            color = random.choice(colors)
        
        # Style-specific interpretation
        style_strokes = {
            "impressionist": {
                "type": "dappled_blobs",
                "description": f"{self.name} dappled {color} in soft, light touches"
            },
            "abstract": {
                "type": "geometric_forms",
                "description": f"{self.name} laid down bold {color} geometric shapes"
            },
            "geometric": {
                "type": "precise_lines",
                "description": f"{self.name} drew precise {color} lines and angles"
            },
            "fluid": {
                "type": "flowing_wash",
                "description": f"{self.name} created flowing {color} washes"
            },
            "chaotic": {
                "type": "splatter",
                "description": f"{self.name} splattered wild {color} across the canvas"
            }
        }
        
        stroke_base = style_strokes.get(self.style, style_strokes["impressionist"])
        
        # Modify based on emotion movement type
        movement = params.get("movement", "flowing")
        movement_descriptions = {
            "circular": "in joyful circles",
            "dripping": "with melancholy drips",
            "jagged": "in angry, sharp movements",
            "flowing": "in serene, flowing motions",
            "swirling": "in wonder-filled spirals",
            "erratic": "in chaotic, unpredictable bursts"
        }
        
        description = f"{stroke_base['description']} {movement_descriptions.get(movement, '')}"
        
        return {
            "type": stroke_base["type"],
            "color": color,
            "description": description,
            "intensity": params.get("intensity", 0.5)
        }
    
    async def refill_paint(self):
        """Refill the robot's paint supply"""
        print(f"   {self.name} refilling paint...")
        await asyncio.sleep(1)
        self.paint_level = 100.0
        print(f"   ✓ {self.name} paint full")
    
    def get_status(self) -> dict:
        """Get current robot status"""
        return {
            "name": self.name,
            "style": self.style,
            "paintings": self.paintings_completed,
            "paint_level": round(self.paint_level, 1),
            "favorite_color": self.favorite_color
        }