"""
Plant Personalities - Each plant is unique
"""
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List


class PlantSpecies(Enum):
    """Different plant types with unique characteristics"""
    FERN = "Fern"
    CACTUS = "Cactus"
    ROSE = "Rose"
    BASIL = "Basil"
    ORCHID = "Orchid"
    SUNFLOWER = "Sunflower"
    TOMATO = "Tomato"
    LAVENDER = "Lavender"


@dataclass
class PlantPersonality:
    """Defines how a plant speaks and behaves"""
    openings: List[str]
    closings: List[str]
    voice_style: str
    
    # Species-specific personalities
    FERN = None
    CACTUS = None
    ROSE = None
    BASIL = None
    ORCHID = None
    SUNFLOWER = None


# Initialize personalities
PlantPersonality.FERN = PlantPersonality(
    openings=[
        "The fern whispers softly:",
        "From shadowed fronds:",
        "In the language of spores:"
    ],
    closings=[
        "...I remain, green and patient.",
        "...shadows are my home.",
        "...humidity is my heart's song."
    ],
    voice_style="shy_poetic"
)

PlantPersonality.CACTUS = PlantPersonality(
    openings=[
        "The cactus proclaims:",
        "From fortress of thorns:",
        "In desert wisdom:"
    ],
    closings=[
        "...water is memory, stored deep.",
        "...I am endurance made flesh.",
        "...the dry world cannot break me."
    ],
    voice_style="stoic_philosopher"
)

PlantPersonality.ROSE = PlantPersonality(
    openings=[
        "The rose declares passionately:",
        "In perfumed drama:",
        "From thorny romance:"
    ],
    closings=[
        "...love me, or let me wilt beautifully.",
        "...beauty demands its price in pain.",
        "...I bloom for admiration alone."
    ],
    voice_style="dramatic_romantic"
)

PlantPersonality.BASIL = PlantPersonality(
    openings=[
        "The basil exclaims cheerfully:",
        "In culinary enthusiasm:",
        "From the herb garden:"
    ],
    closings=[
        "...pick me, taste me, love me in pesto!",
        "...I am flavor waiting to happen.",
        "...cook with joy, season with me!"
    ],
    voice_style="enthusiastic_chef"
)

PlantPersonality.ORCHID = PlantPersonality(
    openings=[
        "The orchid states elegantly:",
        "In aristocratic tones:",
        "From exotic heights:"
    ],
    closings=[
        "...I require precision, not mere care.",
        "...beauty this rare demands effort.",
        "...I am orchid, mysteriously perfect."
    ],
    voice_style="mysterious_aristocrat"
)

PlantPersonality.SUNFLOWER = PlantPersonality(
    openings=[
        "The sunflower beams:",
        "In golden optimism:",
        "Following the light:"
    ],
    closings=[
        "...turn your face to the sun with me!",
        "...every day is a good day to grow!",
        "...I am sunshine made solid!"
    ],
    voice_style="eternal_optimist"
)


class Plant:
    """
    An individual plant with personality, needs, and history.
    """
    
    def __init__(self, name: str, species: PlantSpecies, description: str):
        self.name = name
        self.species = species
        self.description = description
        self.personality = getattr(PlantPersonality, species.name)
        
        # Sensor readings (0-100 scale)
        self.moisture = random.uniform(40, 80)
        self.light = random.uniform(30, 90)
        self.temperature = random.uniform(18, 28)
        self.nutrients = random.uniform(50, 100)
        
        # Care history
        self.care_history = []
        self.current_need = None
        
    def needs_attention(self) -> bool:
        """Check if plant needs care"""
        # Thresholds for needs
        needs = []
        
        if self.moisture < 30:
            needs.append("water")
        if self.light < 20 or self.light > 90:
            needs.append("light")
        if self.temperature < 15 or self.temperature > 35:
            needs.append("temperature")
        if self.nutrients < 20:
            needs.append("fertilizer")
            
        if needs:
            self.current_need = random.choice(needs)
            return True
        
        self.current_need = None
        return False
    
    def receive_care(self, care_type: str):
        """Update state after receiving care"""
        self.care_history.append(care_type)
        
        if care_type == "water":
            self.moisture = min(100, self.moisture + 40)
        elif care_type == "light_adjust":
            self.light = 60
        elif care_type == "temperature":
            self.temperature = 22
        elif care_type == "fertilizer":
            self.nutrients = min(100, self.nutrients + 50)
    
    def __str__(self):
        return f"{self.name} ({self.species.value})"


import random