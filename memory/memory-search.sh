#!/bin/bash
# Memory Search Helper
# Quick search across all memory files

SEARCH_DIR="$HOME/.openclaw/workspace/memory"

if [ $# -eq 0 ]; then
  echo "Usage: memory-search <query>"
  echo ""
  echo "Searches across:"
  echo "  - MEMORY.md (long-term memory)"
  echo "  - daily/*.md (daily notes)"
  echo "  - projects/*.md (project notes)"
  echo ""
  echo "Examples:"
  echo "  memory-search 'Agent ROS Bridge'"
  echo "  memory-search 'TODO'"
  echo "  memory-search '2026-02'"
  exit 1
fi

QUERY="$1"
echo "🔍 Searching memory for: $QUERY"
echo ""

# Search MEMORY.md first (most important)
if [ -f "$SEARCH_DIR/MEMORY.md" ]; then
  RESULTS=$(grep -i -n "$QUERY" "$SEARCH_DIR/MEMORY.md" 2>/dev/null)
  if [ -n "$RESULTS" ]; then
    echo "📚 MEMORY.md (Long-term):"
    echo "$RESULTS" | head -10
    echo ""
  fi
fi

# Search daily notes (recent context)
DAILY_RESULTS=$(find "$SEARCH_DIR/daily" -name "*.md" -exec grep -l -i "$QUERY" {} \; 2>/dev/null | sort -r | head -5)
if [ -n "$DAILY_RESULTS" ]; then
  echo "📅 Daily Notes:"
  for file in $DAILY_RESULTS; do
    DATE=$(basename "$file" .md)
    echo "  - $DATE"
  done
  echo ""
fi

# Search project notes
PROJECT_RESULTS=$(find "$SEARCH_DIR/projects" -name "*.md" -exec grep -l -i "$QUERY" {} \; 2>/dev/null)
if [ -n "$PROJECT_RESULTS" ]; then
  echo "🗂️  Projects:"
  for file in $PROJECT_RESULTS; do
    PROJECT=$(basename "$file" .md)
    echo "  - $PROJECT"
  done
  echo ""
fi

echo "✅ Search complete"
