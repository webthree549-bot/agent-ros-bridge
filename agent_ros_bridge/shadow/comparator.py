"""Decision comparator for AI-human agreement analysis."""

from typing import Any

from .models import AIProposal, DecisionRecord, HumanAction


class DecisionComparator:
    """Compare AI proposals with human actions."""

    INTENT_SYNONYMS = {
        "NAVIGATE": ["go", "move", "drive", "navigate", "head", "proceed"],
        "MANIPULATE": ["pick", "grab", "place", "put", "drop", "manipulate"],
        "SENSE": ["see", "scan", "look", "detect", "observe", "sense"],
        "QUERY": ["status", "where", "what", "battery", "query"],
        "SAFETY": ["stop", "emergency", "halt", "safety"],
        "CONFIGURE": ["set", "enable", "disable", "configure", "mode"],
    }

    def __init__(
        self,
        intent_match_threshold: float = 0.8,
        entity_match_threshold: float = 0.7,
    ):
        """Initialize comparator."""
        self._intent_threshold = intent_match_threshold
        self._entity_threshold = entity_match_threshold

    def compare(
        self,
        record: DecisionRecord,
    ) -> tuple[bool, float]:
        """Compare AI proposal with human action in a record."""
        if record.ai_proposal is None or record.human_action is None:
            return False, 0.0

        intent_score = self._compare_intents(record.ai_proposal, record.human_action)
        entity_score = self._compare_entities(record.ai_proposal, record.human_action)
        total_score = (intent_score * 0.6) + (entity_score * 0.4)

        agrees = (
            intent_score >= self._intent_threshold
            and entity_score >= self._entity_threshold
        )

        return agrees, total_score

    def _compare_intents(
        self,
        ai_proposal: AIProposal,
        human_action: HumanAction,
    ) -> float:
        """Compare intent types."""
        ai_intent = ai_proposal.intent_type.upper()
        human_command = human_action.command.upper()

        if ai_intent in human_command or human_command in ai_intent:
            return 1.0

        ai_synonyms = self.INTENT_SYNONYMS.get(ai_intent, [])
        for synonym in ai_synonyms:
            if synonym.upper() in human_command:
                return 0.9

        human_words = set(human_command.split())
        ai_words = set(ai_intent.split())
        ai_synonym_words = set()
        for syn in ai_synonyms:
            ai_synonym_words.update(syn.upper().split())

        all_ai_words = ai_words | ai_synonym_words
        if human_words & all_ai_words:
            return 0.7

        return 0.0

    def _compare_entities(
        self,
        ai_proposal: AIProposal,
        human_action: HumanAction,
    ) -> float:
        """Compare extracted entities."""
        ai_entities = ai_proposal.entities
        human_params = human_action.parameters

        if not ai_entities and not human_params:
            return 1.0

        if not ai_entities or not human_params:
            return 0.3

        ai_values = set()
        for entity in ai_entities:
            value = entity.get("value", "").upper()
            if value:
                ai_values.add(value)

        human_values = set()
        for key, value in human_params.items():
            if isinstance(value, str):
                human_values.add(value.upper())
            elif isinstance(value, (int, float)):
                human_values.add(str(value).upper())

        if not ai_values or not human_values:
            return 0.5 if (ai_values or human_values) else 1.0

        intersection = ai_values & human_values
        union = ai_values | human_values

        if not union:
            return 1.0

        jaccard_score = len(intersection) / len(union)
        return max(0.5, jaccard_score) if intersection else jaccard_score

    def calculate_metrics(
        self,
        records: list[DecisionRecord],
    ) -> dict[str, Any]:
        """Calculate agreement metrics."""
        if not records:
            return {
                "total": 0,
                "agreement_count": 0,
                "agreement_rate": 0.0,
                "average_score": 0.0,
                "high_confidence_agreement": 0.0,
            }

        results = [self.compare(r) for r in records]
        agreements = [r[0] for r in results]
        scores = [r[1] for r in results]

        agreement_count = sum(agreements)
        total = len(records)

        high_confidence_count = 0
        for record, (agrees, _) in zip(records, results, strict=False):
            ai_confident = record.ai_proposal and record.ai_proposal.confidence >= 0.95
            if agrees and ai_confident:
                high_confidence_count += 1

        return {
            "total": total,
            "agreement_count": agreement_count,
            "agreement_rate": agreement_count / total if total > 0 else 0.0,
            "average_score": sum(scores) / len(scores) if scores else 0.0,
            "high_confidence_agreement": high_confidence_count / total if total > 0 else 0.0,
            "score_distribution": {
                "0.0-0.3": sum(1 for s in scores if s < 0.3),
                "0.3-0.5": sum(1 for s in scores if 0.3 <= s < 0.5),
                "0.5-0.7": sum(1 for s in scores if 0.5 <= s < 0.7),
                "0.7-0.9": sum(1 for s in scores if 0.7 <= s < 0.9),
                "0.9-1.0": sum(1 for s in scores if s >= 0.9),
            },
        }
