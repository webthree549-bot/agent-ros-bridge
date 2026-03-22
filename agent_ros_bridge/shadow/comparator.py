"""Decision comparator for AI-human agreement analysis.

Compares AI proposals against human actions to calculate
agreement metrics for shadow mode validation.
"""

from typing import Any

from .models import AIProposal, DecisionRecord, HumanAction


class DecisionComparator:
    """Compare AI proposals with human actions.

    Usage:
        comparator = DecisionComparator()

        # Compare a record
        agreement, score = comparator.compare(record)

        # Batch compare
        results = comparator.compare_batch(records)

        # Get metrics
        metrics = comparator.calculate_metrics(records)
    """

    # Intent type synonyms for fuzzy matching
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
        """Initialize comparator.

        Args:
            intent_match_threshold: Minimum score for intent agreement
            entity_match_threshold: Minimum score for entity agreement
        """
        self._intent_threshold = intent_match_threshold
        self._entity_threshold = entity_match_threshold

    def compare(
        self,
        record: DecisionRecord,
    ) -> tuple[bool, float]:
        """Compare AI proposal with human action in a record.

        Args:
            record: Decision record with both AI and human decisions

        Returns:
            (agreement, score): Whether they agree and similarity score (0.0-1.0)
        """
        if record.ai_proposal is None or record.human_action is None:
            return False, 0.0

        # Calculate intent match score
        intent_score = self._compare_intents(
            record.ai_proposal,
            record.human_action,
        )

        # Calculate entity match score
        entity_score = self._compare_entities(
            record.ai_proposal,
            record.human_action,
        )

        # Weighted combination (intent is more important)
        total_score = (intent_score * 0.6) + (entity_score * 0.4)

        # Agreement requires both scores above thresholds
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
        """Compare intent types.

        Returns:
            score: 0.0-1.0 similarity score
        """
        ai_intent = ai_proposal.intent_type.upper()
        human_command = human_action.command.upper()

        # Exact match
        if ai_intent in human_command or human_command in ai_intent:
            return 1.0

        # Check synonyms
        ai_synonyms = self.INTENT_SYNONYMS.get(ai_intent, [])
        for synonym in ai_synonyms:
            if synonym.upper() in human_command:
                return 0.9

        # Check if human command contains any AI intent words
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
        """Compare extracted entities.

        Returns:
            score: 0.0-1.0 similarity score
        """
        ai_entities = ai_proposal.entities
        human_params = human_action.parameters

        if not ai_entities and not human_params:
            return 1.0  # Both empty = match

        if not ai_entities or not human_params:
            return 0.3  # One empty, one not = partial

        # Extract values from AI entities
        ai_values = set()
        for entity in ai_entities:
            value = entity.get("value", "").upper()
            if value:
                ai_values.add(value)

        # Extract values from human parameters
        human_values = set()
        for key, value in human_params.items():
            if isinstance(value, str):
                human_values.add(value.upper())
            elif isinstance(value, (int, float)):
                human_values.add(str(value).upper())

        if not ai_values or not human_values:
            return 0.5 if (ai_values or human_values) else 1.0

        # Calculate overlap
        intersection = ai_values & human_values
        union = ai_values | human_values

        if not union:
            return 1.0

        jaccard_score = len(intersection) / len(union)

        # Boost score for partial matches
        if intersection:
            return max(0.5, jaccard_score)

        return jaccard_score

    def compare_batch(
        self,
        records: list[DecisionRecord],
    ) -> list[tuple[bool, float]]:
        """Compare multiple records.

        Args:
            records: List of decision records

        Returns:
            List of (agreement, score) tuples
        """
        return [self.compare(r) for r in records]

    def calculate_metrics(
        self,
        records: list[DecisionRecord],
    ) -> dict[str, Any]:
        """Calculate agreement metrics.

        Args:
            records: List of completed decision records

        Returns:
            Metrics dictionary
        """
        if not records:
            return {
                "total": 0,
                "agreement_count": 0,
                "agreement_rate": 0.0,
                "average_score": 0.0,
                "high_confidence_agreement": 0.0,
            }

        results = self.compare_batch(records)

        agreements = [r[0] for r in results]
        scores = [r[1] for r in results]

        agreement_count = sum(agreements)
        total = len(records)

        # High confidence = both AI and comparator agree
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

    def get_disagreements(
        self,
        records: list[DecisionRecord],
    ) -> list[tuple[DecisionRecord, float]]:
        """Get records where AI and human disagreed.

        Args:
            records: List of decision records

        Returns:
            List of (record, score) tuples where agreement is False
        """
        results = []
        for record in records:
            agrees, score = self.compare(record)
            if not agrees:
                results.append((record, score))
        return results
