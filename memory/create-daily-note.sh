#!/bin/bash
# Daily Note Creator
# Creates a new daily note if it doesn't exist

DATE=$(date +%Y-%m-%d)
NOTE_FILE="$HOME/.openclaw/workspace/memory/daily/$DATE.md"

if [ ! -f "$NOTE_FILE" ]; then
  cat > "$NOTE_FILE" << EOF
# 📝 Daily Note: $DATE

**Day:** $(date +%A)  
**Time:** $(date +"%I:%M %p %Z")  
**Location:** ~/.openclaw/workspace

---

## 🎯 Today's Focus

1. 
2. 
3. 

---

## ✅ Completed

- [ ] 
- [ ] 
- [ ] 

---

## 🚧 In Progress

- [ ] 

---

## 📝 Observations



---

## 🔗 References

- **MEMORY.md:** ../MEMORY.md
- **Yesterday:** ./$(date -v-1d +%Y-%m-%d 2>/dev/null || date -d "yesterday" +%Y-%m-%d 2>/dev/null || echo "N/A").md

---

*Created: $(date "+%Y-%m-%d %H:%M %Z")*
EOF
  echo "✅ Created daily note: $NOTE_FILE"
else
  echo "📓 Daily note already exists: $NOTE_FILE"
fi
