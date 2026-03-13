#!/usr/bin/env python3
"""
Multi-Language Support for Intent Parser
Agent ROS Bridge v0.6.1 - Week 6 Advanced Features

Supports multiple languages for intent parsing:
- English (en) - Primary
- Spanish (es)
- French (fr)
- German (de)
- Chinese (zh)
- Japanese (ja)

Uses translation + pattern matching or native patterns.
"""

import re
from dataclasses import dataclass
from typing import Any


@dataclass
class LanguagePatterns:
    """Patterns for a specific language."""

    language_code: str
    language_name: str
    patterns: dict[str, list[str]]
    entity_patterns: dict[str, list[str]]


class MultiLanguageParser:
    """
    Multi-language intent parser.

    Supports intent parsing in multiple languages through:
    1. Native regex patterns (for supported languages)
    2. Translation + English patterns (fallback)
    """

    # Language-specific patterns
    LANGUAGE_PATTERNS = {
        "en": LanguagePatterns(
            language_code="en",
            language_name="English",
            patterns={
                "NAVIGATE": [
                    r"go\s+to\s+(?:the\s+)?(?P<location>\w+)",
                    r"navigate\s+to\s+(?:the\s+)?(?P<location>\w+)",
                    r"move\s+to\s+(?:the\s+)?(?P<location>\w+)",
                ],
                "MANIPULATE": [
                    r"pick\s+up\s+(?:the\s+)?(?P<object>\w+)",
                    r"grab\s+(?:the\s+)?(?P<object>\w+)",
                    r"place\s+(?:the\s+)?(?P<object>\w+)",
                ],
                "SENSE": [
                    r"what\s+do\s+you\s+see",
                    r"scan\s+(?:the\s+)?(?P<area>\w+)",
                ],
                "SAFETY": [
                    r"^stop$",
                    r"emergency\s+stop",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:to|at|in)\s+(?:the\s+)?(\w+)"],
                "OBJECT": [r"(?:the|a|an)\s+(\w+)"],
            },
        ),
        "es": LanguagePatterns(
            language_code="es",
            language_name="Spanish",
            patterns={
                "NAVIGATE": [
                    r"ve\s+a\s+(?:la\s+|el\s+)?(?P<location>\w+)",
                    r"ir\s+a\s+(?:la\s+|el\s+)?(?P<location>\w+)",
                    r"navegar\s+a\s+(?:la\s+|el\s+)?(?P<location>\w+)",
                ],
                "MANIPULATE": [
                    r"recoger\s+(?:la\s+|el\s+)?(?P<object>\w+)",
                    r"tomar\s+(?:la\s+|el\s+)?(?P<object>\w+)",
                    r"poner\s+(?:la\s+|el\s+)?(?P<object>\w+)",
                ],
                "SENSE": [
                    r"qué\s+ves",
                    r"escanear\s+(?:la\s+|el\s+)?(?P<area>\w+)",
                ],
                "SAFETY": [
                    r"^para$",
                    r"alto\s+de\s+emergencia",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:a|en)\s+(?:la\s+|el\s+)?(\w+)"],
                "OBJECT": [r"(?:la\s+|el\s+|una\s+|un\s+)?(\w+)"],
            },
        ),
        "fr": LanguagePatterns(
            language_code="fr",
            language_name="French",
            patterns={
                "NAVIGATE": [
                    r"va\s+(?:à\s+|au\s+)?(?P<location>\w+)",
                    r"aller\s+(?:à\s+|au\s+)?(?P<location>\w+)",
                    r"naviguer\s+(?:à\s+|au\s+)?(?P<location>\w+)",
                ],
                "MANIPULATE": [
                    r"prendre\s+(?:la\s+|le\s+)?(?P<object>\w+)",
                    r"ramasser\s+(?:la\s+|le\s+)?(?P<object>\w+)",
                    r"poser\s+(?:la\s+|le\s+)?(?P<object>\w+)",
                ],
                "SENSE": [
                    r"que\s+vois-tu",
                    r"scanner\s+(?:la\s+|le\s+)?(?P<area>\w+)",
                ],
                "SAFETY": [
                    r"^arrête$",
                    r"arrêt\s+d'urgence",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:à|au|dans)\s+(?:la\s+|le\s+)?(\w+)"],
                "OBJECT": [r"(?:la\s+|le\s+|une\s+|un\s+)?(\w+)"],
            },
        ),
        "de": LanguagePatterns(
            language_code="de",
            language_name="German",
            patterns={
                "NAVIGATE": [
                    r"geh\s+(?:zur\s+|zum\s+)?(?P<location>\w+)",
                    r"fahre\s+(?:zur\s+|zum\s+)?(?P<location>\w+)",
                    r"navigiere\s+(?:zur\s+|zum\s+)?(?P<location>\w+)",
                ],
                "MANIPULATE": [
                    r"heb\s+(?:die\s+|den\s+)?(?P<object>\w+)\s+auf",
                    r"nimm\s+(?:die\s+|den\s+)?(?P<object>\w+)",
                    r"stell\s+(?:die\s+|den\s+)?(?P<object>\w+)\s+ab",
                ],
                "SENSE": [
                    r"was\s+siehst\s+du",
                    r"scanne\s+(?:die\s+|den\s+)?(?P<area>\w+)",
                ],
                "SAFETY": [
                    r"^stopp$",
                    r"notstopp",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:zur|zum|in\s+(?:die|den))\s+(\w+)"],
                "OBJECT": [r"(?:die\s+|den\s+|eine\s+|einen\s+)?(\w+)"],
            },
        ),
        "zh": LanguagePatterns(
            language_code="zh",
            language_name="Chinese",
            patterns={
                "NAVIGATE": [
                    r"去(?:到)?(?P<location>\w+)",
                    r"前往(?P<location>\w+)",
                    r"移动到(?P<location>\w+)",
                ],
                "MANIPULATE": [
                    r"拿起(?P<object>\w+)",
                    r"抓取(?P<object>\w+)",
                    r"放下(?P<object>\w+)",
                ],
                "SENSE": [
                    r"你看到了什么",
                    r"扫描(?P<area>\w+)",
                ],
                "SAFETY": [
                    r"^停止$",
                    r"紧急停止",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:去|到|在)(\w+)"],
                "OBJECT": [r"(?:个|只|张)?(\w+)"],
            },
        ),
        "ja": LanguagePatterns(
            language_code="ja",
            language_name="Japanese",
            patterns={
                "NAVIGATE": [
                    r"(?P<location>\w+)(?:に|へ)行って",
                    r"(?P<location>\w+)(?:に|へ)移動",
                    r"(?P<location>\w+)をナビゲート",
                ],
                "MANIPULATE": [
                    r"(?P<object>\w+)を取って",
                    r"(?P<object>\w+)を持ち上げて",
                    r"(?P<object>\w+)を置いて",
                ],
                "SENSE": [
                    r"何が見える",
                    r"(?P<area>\w+)をスキャン",
                ],
                "SAFETY": [
                    r"^止まれ$",
                    r"非常停止",
                ],
            },
            entity_patterns={
                "LOCATION": [r"(?:に|へ|で)(\w+)"],
                "OBJECT": [r"(\w+)(?:を|が)"],
            },
        ),
    }

    def __init__(self, default_language: str = "en"):
        """
        Initialize multi-language parser.

        Args:
            default_language: Default language code (e.g., 'en', 'es')
        """
        self._default_language = default_language
        self._compiled_patterns: dict[str, dict[str, list[re.Pattern]]] = {}

        # Compile patterns for all languages
        self._compile_all_patterns()

    def _compile_all_patterns(self):
        """Compile regex patterns for all languages."""
        for lang_code, lang_patterns in self.LANGUAGE_PATTERNS.items():
            self._compiled_patterns[lang_code] = {"intents": {}, "entities": {}}

            # Compile intent patterns
            for intent_type, patterns in lang_patterns.patterns.items():
                self._compiled_patterns[lang_code]["intents"][intent_type] = [
                    re.compile(p, re.IGNORECASE) for p in patterns
                ]

            # Compile entity patterns
            for entity_type, patterns in lang_patterns.entity_patterns.items():
                self._compiled_patterns[lang_code]["entities"][entity_type] = [
                    re.compile(p, re.IGNORECASE) for p in patterns
                ]

    def detect_language(self, utterance: str) -> str:
        """
        Detect language of utterance.

        Simple heuristic based on character ranges.
        For production, consider using a proper language detection library.

        Args:
            utterance: Input utterance

        Returns:
            Language code (e.g., 'en', 'es', 'zh')
        """
        # Check for Chinese characters
        if any("\u4e00" <= c <= "\u9fff" for c in utterance):
            return "zh"

        # Check for Japanese Hiragana/Katakana
        if any("\u3040" <= c <= "\u309f" or "\u30a0" <= c <= "\u30ff" for c in utterance):
            return "ja"

        # Check for Spanish-specific patterns
        spanish_markers = ["á", "é", "í", "ó", "ú", "ñ", "¿", "¡"]
        if any(m in utterance.lower() for m in spanish_markers):
            return "es"

        # Check for French-specific patterns
        french_markers = ["ç", "è", "à", "ù", "ê", "ô"]
        if any(m in utterance.lower() for m in french_markers):
            return "fr"

        # Check for German-specific patterns
        german_markers = ["ß", "ä", "ö", "ü"]
        if any(m in utterance.lower() for m in german_markers):
            return "de"

        # Default to English
        return "en"

    def parse(self, utterance: str, language: str | None = None) -> dict[str, Any] | None:
        """
        Parse utterance in specified or detected language.

        Args:
            utterance: Natural language input
            language: Language code (auto-detect if None)

        Returns:
            Parsed intent or None
        """
        if language is None:
            language = self.detect_language(utterance)

        # Fall back to default if language not supported
        if language not in self.LANGUAGE_PATTERNS:
            language = self._default_language

        patterns = self._compiled_patterns[language]
        lang_data = self.LANGUAGE_PATTERNS[language]

        # Try to match intent patterns
        for intent_type, intent_patterns in patterns["intents"].items():
            for pattern in intent_patterns:
                match = pattern.search(utterance)
                if match:
                    entities = []

                    # Extract entities from named groups
                    for key, value in match.groupdict().items():
                        entity_type = self._map_entity_type(key, lang_data)
                        entities.append({"type": entity_type, "value": value, "confidence": 0.95})

                    return {
                        "intent_type": intent_type,
                        "confidence": 0.95,
                        "entities": entities,
                        "language": language,
                        "language_name": lang_data.language_name,
                    }

        return None

    def _map_entity_type(self, group_name: str, lang_data: LanguagePatterns) -> str:
        """Map regex group name to entity type."""
        mapping = {
            "location": "LOCATION",
            "object": "OBJECT",
            "area": "LOCATION",
        }
        return mapping.get(group_name, group_name.upper())

    def get_supported_languages(self) -> list[dict[str, str]]:
        """Get list of supported languages."""
        return [
            {"code": code, "name": data.language_name}
            for code, data in self.LANGUAGE_PATTERNS.items()
        ]


def main():
    """Test multi-language parser."""
    parser = MultiLanguageParser()

    print("Multi-Language Intent Parser")
    print(f"Supported languages: {[l['name'] for l in parser.get_supported_languages()]}\n")

    test_utterances = [
        ("go to kitchen", None),
        ("ve a la cocina", None),  # Spanish
        ("va à la cuisine", None),  # French
        ("geh zur küche", None),  # German
        ("去厨房", None),  # Chinese
        ("キッチンに行って", None),  # Japanese
    ]

    for utterance, lang in test_utterances:
        detected = parser.detect_language(utterance)
        result = parser.parse(utterance, lang)

        print(f"Utterance: '{utterance}'")
        print(f"  Detected language: {detected}")

        if result:
            print(f"  Intent: {result['intent_type']}")
            print(f"  Confidence: {result['confidence']}")
            print(f"  Entities: {result['entities']}")
        else:
            print("  No match found")

        print()


if __name__ == "__main__":
    main()
