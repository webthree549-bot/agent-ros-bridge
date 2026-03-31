import time


def demo_performance():
    """Demonstrate performance metrics."""
    print("Performance Metrics")

    # Use rule-based parser that doesn't require ROS
    try:
        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        from agent_ros_bridge_msgs.srv import ParseIntent

        parser = IntentParserNode()
        use_ros = True
    except ImportError:
        # Fallback to simple regex-based parsing for demo
        import re

        use_ros = False

    # Measure parsing latency
    latencies = []
    test_utterances = [
        "go to kitchen",
        "pick up the cup",
        "stop",
        "navigate to position 1 2",
    ]

    for i in range(100):
        utterance = test_utterances[i % len(test_utterances)]

        start = time.time()

        if use_ros:
            request = ParseIntent.Request()
            request.utterance = utterance
            response = ParseIntent.Response()
            parser.parse_intent_callback(request, response)
        else:
            # Simple regex parsing for demo
            import re

            if re.search(r"\bgo\s+to\b|\bnavigate\b", utterance, re.I):
                intent_type = "NAVIGATE"
            elif re.search(r"\bpick\s+up\b|\bgrab\b", utterance, re.I):
                intent_type = "MANIPULATE"
            elif re.search(r"\bstop\b|\bhalt\b", utterance, re.I):
                intent_type = "SAFETY"
            else:
                intent_type = "UNKNOWN"

        latency = (time.time() - start) * 1000
        latencies.append(latency)

    avg_latency = sum(latencies) / len(latencies)
    max_latency = max(latencies)
    min_latency = min(latencies)

    print("  Intent Parsing Performance (100 iterations):")
    print(f"    → Average: {avg_latency:.2f}ms")
    print(f"    → Min: {min_latency:.2f}ms")
    print(f"    → Max: {max_latency:.2f}ms")

    if avg_latency < 10:
        print("    → Target: <10ms ✅")
    else:
        print("    → Target: <10ms ❌ (consider optimization)")

    if not use_ros:
        print("    (Using rule-based parser for demo)")
