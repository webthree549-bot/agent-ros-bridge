"""
Emotion Engine - Translates emotions into artistic parameters
"""
import random


class EmotionAnalyzer:
    """
    Converts emotional states into painting parameters.
    Like synesthesia for robots.
    """
    
    EMOTION_PALETTES = {
        "joy": {
            "colors": ["#FFD700", "#FFA500", "#FF6B35", "#FFE66D"],
            "description": "Warm golds and oranges, bursting with light"
        },
        "sadness": {
            "colors": ["#4A5568", "#718096", "#A0AEC0", "#2D3748"],
            "description": "Cool greys and blues, muted tones"
        },
        "anger": {
            "colors": ["#DC2626", "#991B1B", "#1A1A1A", "#FF4444"],
            "description": "Violent reds and stark blacks"
        },
        "calm": {
            "colors": ["#48BB78", "#81E6D9", "#B794F4", "#E6FFFA"],
            "description": "Soft greens, teals, and pastels"
        },
        "wonder": {
            "colors": ["#9F7AEA", "#F6AD55", "#4FD1C5", "#F6E05E"],
            "description": "Purples, golds, and iridescent teals"
        },
        "chaos": {
            "colors": ["#FF006E", "#FB5607", "#FFBE0B", "#8338EC"],
            "description": "Clashing neons, unpredictable"
        }
    }
    
    MOVEMENT_PATTERNS = {
        "joy": {"type": "circular", "energy": 0.8, "fluidity": 0.9},
        "sadness": {"type": "dripping", "energy": 0.2, "fluidity": 0.4},
        "anger": {"type": "jagged", "energy": 1.0, "fluidity": 0.1},
        "calm": {"type": "flowing", "energy": 0.3, "fluidity": 1.0},
        "wonder": {"type": "swirling", "energy": 0.6, "fluidity": 0.8},
        "chaos": {"type": "erratic", "energy": 0.9, "fluidity": 0.2}
    }
    
    def emotion_to_params(self, emotion: str) -> dict:
        """
        Convert emotion string into painting parameters.
        
        Returns dict with:
            - colors: list of hex colors
            - movement: movement pattern type
            - intensity: 0-1 energy level
            - fluidity: 0-1 smoothness
            - brush_size: relative brush size
            - speed: painting speed multiplier
        """
        palette = self.EMOTION_PALETTES.get(emotion, self.EMOTION_PALETTES["calm"])
        movement = self.MOVEMENT_PATTERNS.get(emotion, self.MOVEMENT_PATTERNS["calm"])
        
        # Add some randomness for organic feel
        variation = random.uniform(0.8, 1.2)
        
        return {
            "emotion": emotion,
            "colors": palette["colors"],
            "color_description": palette["description"],
            "movement": movement["type"],
            "intensity": round(movement["energy"] * variation, 2),
            "fluidity": round(movement["fluidity"] * variation, 2),
            "brush_size": self._get_brush_size(emotion),
            "speed": self._get_speed(emotion),
            "layering": self._get_layering(emotion)
        }
    
    def _get_brush_size(self, emotion: str) -> str:
        sizes = {
            "joy": "bold_large",
            "sadness": "fine_delicate",
            "anger": "heavy_impasto",
            "calm": "medium_soft",
            "wonder": "varied_expressive",
            "chaos": "random_mixed"
        }
        return sizes.get(emotion, "medium")
    
    def _get_speed(self, emotion: str) -> float:
        speeds = {
            "joy": 1.5,
            "sadness": 0.4,
            "anger": 2.0,
            "calm": 0.6,
            "wonder": 1.0,
            "chaos": 1.8
        }
        return speeds.get(emotion, 1.0)
    
    def _get_layering(self, emotion: str) -> int:
        layers = {
            "joy": 3,
            "sadness": 5,
            "anger": 2,
            "calm": 4,
            "wonder": 6,
            "chaos": 8
        }
        return layers.get(emotion, 3)
    
    def blend_emotions(self, emotion1: str, emotion2: str, ratio: float = 0.5) -> dict:
        """
        Create parameters that blend two emotions.
        Useful for emotional transitions.
        """
        params1 = self.emotion_to_params(emotion1)
        params2 = self.emotion_to_params(emotion2)
        
        # Blend colors
        num_colors = max(len(params1["colors"]), len(params2["colors"]))
        blended_colors = []
        for i in range(num_colors):
            c1 = params1["colors"][i % len(params1["colors"])]
            c2 = params2["colors"][i % len(params2["colors"])]
            blended_colors.append(self._blend_hex_colors(c1, c2, ratio))
        
        return {
            "emotion": f"{emotion1}-{emotion2}",
            "colors": blended_colors,
            "intensity": round(params1["intensity"] * (1-ratio) + params2["intensity"] * ratio, 2),
            "fluidity": round(params1["fluidity"] * (1-ratio) + params2["fluidity"] * ratio, 2),
            "is_blend": True
        }
    
    def _blend_hex_colors(self, c1: str, c2: str, ratio: float) -> str:
        """Simple hex color blending"""
        c1 = c1.lstrip('#')
        c2 = c2.lstrip('#')
        
        r = int(int(c1[0:2], 16) * (1-ratio) + int(c2[0:2], 16) * ratio)
        g = int(int(c1[2:4], 16) * (1-ratio) + int(c2[2:4], 16) * ratio)
        b = int(int(c1[4:6], 16) * (1-ratio) + int(c2[4:6], 16) * ratio)
        
        return f"#{r:02x}{g:02x}{b:02x}"
    
    def analyze_input(self, input_text: str) -> str:
        """
        Simple sentiment analysis to detect emotion from text.
        In production, this would use a proper NLP model.
        """
        text = input_text.lower()
        
        emotion_keywords = {
            "joy": ["happy", "joy", "excited", "wonderful", "amazing", "great", "love", "🎉", "😊"],
            "sadness": ["sad", "depressed", "melancholy", "blue", "grief", "tears", "😢", "💔"],
            "anger": ["angry", "furious", "mad", "rage", "hate", "frustrated", "😠", "🔥"],
            "calm": ["peaceful", "calm", "serene", "relaxed", "gentle", "quiet", "🧘", "🌊"],
            "wonder": ["wow", "amazing", "incredible", "magical", "beautiful", "✨", "🌟"],
            "chaos": ["crazy", "wild", "chaotic", "confused", "overwhelmed", "🌀", "🌪️"]
        }
        
        scores = {emotion: 0 for emotion in emotion_keywords}
        
        for emotion, keywords in emotion_keywords.items():
            for keyword in keywords:
                if keyword in text:
                    scores[emotion] += 1
        
        # Return highest scoring emotion, default to wonder if no match
        if max(scores.values()) == 0:
            return random.choice(list(emotion_keywords.keys()))
        
        return max(scores, key=scores.get)