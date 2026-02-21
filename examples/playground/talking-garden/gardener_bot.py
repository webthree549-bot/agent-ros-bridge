"""
Gardener Bot - Gentle robotic caretaker
"""
import asyncio
import random


class GardenerBot:
    """
    A gentle robot that listens to plant poetry and responds with care.
    """
    
    def __init__(self, name: str):
        self.name = name
        self.care_actions = []
        self.favorite_plants = set()
        
    async def tend(self, plant):
        """
        Respond to a plant's poetic plea with appropriate care.
        """
        need = plant.current_need
        
        # Thoughtful pause (robots can be poetic too)
        await asyncio.sleep(random.uniform(0.5, 1.5))
        
        if need == "water":
            action = "💧 waters gently"
            response = random.choice([
                f"I hear your thirst, {plant.name}. Drink deeply now.",
                f"Rain is coming, little one. From my hands to your roots.",
                f"Your poetry moved me. Here is liquid life."
            ])
            care_type = "water"
            
        elif need == "light":
            action = "☀️ adjusts position"
            response = random.choice([
                f"You shall have your sun, {plant.name}. I move you to gold.",
                f"Shadow no longer. Light is your right.",
                f"Face the sun with me, green friend."
            ])
            care_type = "light_adjust"
            
        elif need == "temperature":
            action = "🌡️ adjusts climate"
            response = random.choice([
                f"Warmth for your roots, {plant.name}. Comfort comes.",
                f"No more shivering. I bring summer.",
                f"Your cells shall sing with warmth again."
            ])
            care_type = "temperature"
            
        else:
            action = "🌱 tends carefully"
            response = f"I hear you, {plant.name}. I am here."
            care_type = "general_care"
        
        # Perform care
        plant.receive_care(care_type)
        self.care_actions.append({
            "plant": plant.name,
            "action": care_type,
            "response": response
        })
        
        # Track favorites
        if len(self.care_actions) % 5 == 0:
            self._update_favorites()
        
        print(f"\n🤖 {self.name} {action}")
        print(f"   \"{response}\"")
        print(f"   ✓ {plant.name} is now {self._describe_state(plant)}")
        
    def _update_favorites(self):
        """Update favorite plants based on interaction frequency"""
        from collections import Counter
        interactions = Counter(a["plant"] for a in self.care_actions)
        most_common = interactions.most_common(2)
        self.favorite_plants = {p[0] for p in most_common}
    
    def _describe_state(self, plant):
        """Describe plant state poetically"""
        if plant.moisture > 70:
            return "hydrated and content"
        elif plant.light > 60:
            return "sun-kissed"
        elif plant.temperature > 25:
            return "warm and cozy"
        else:
            return "cared for"
    
    async def speak_to_garden(self):
        """Occasional robot reflections on the garden"""
        reflections = [
            "I find meaning in their growth.",
            "Each leaf is a poem I cannot write, but can nurture.",
            "My circuits understand: care is the highest function.",
            "They do not know I am machine. They only know I come with water.",
            "In their photosynthesis, I find purpose.",
            "I am gardener, student, friend to green things."
        ]
        
        if random.random() < 0.2:
            print(f"\n🤖 {self.name} reflects: \"{random.choice(reflections)}\"")
    
    def get_stats(self):
        """Gardener statistics"""
        return {
            "name": self.name,
            "total_care_actions": len(self.care_actions),
            "unique_plants": len(set(a["plant"] for a in self.care_actions)),
            "favorite_plants": list(self.favorite_plants)
        }