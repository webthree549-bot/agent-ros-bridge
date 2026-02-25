# 🧠 Memory System

**Persistent context for OpenClaw. Never lose track of what matters.**

---

## 📁 Structure

```
memory/
├── MEMORY.md              # Long-term curated knowledge
├── README.md              # This file
├── create-daily-note.sh   # Daily note automation
├── memory-search.sh       # Search helper
├── daily/                 # Daily context
│   ├── 2026-02-24.md
│   └── YYYY-MM-DD.md
├── projects/              # Project-specific notes
│   └── agent-ros-bridge.md
└── archive/               # Old daily notes (>30 days)
```

---

## 🚀 Quick Start

### Create Today's Note
```bash
~/.openclaw/workspace/memory/create-daily-note.sh
```

Or it's automatic via cron at 6 AM daily.

### Search Memory
```bash
~/.openclaw/workspace/memory/memory-search.sh "Agent ROS Bridge"
```

### Edit Long-term Memory
```bash
# Important knowledge that persists
~/.openclaw/workspace/memory/MEMORY.md
```

---

## 📋 File Types

### MEMORY.md
**Purpose:** Long-term curated knowledge  
**Updated:** Manually when important decisions are made  
**Contains:**
- Core knowledge
- Important decisions
- User preferences
- Quick links

### daily/YYYY-MM-DD.md
**Purpose:** Daily context and todos  
**Updated:** Daily (auto-created at 6 AM)  
**Contains:**
- Today's focus
- Completed tasks
- Observations
- References

### projects/*.md
**Purpose:** Project-specific context  
**Updated:** As projects evolve  
**Contains:**
- Project overview
- Current status
- Key commands
- Release history

---

## ⚙️ Automation

### Cron Jobs

```bash
# Daily at 6 AM: Create new daily note
0 6 * * * /Users/webthree/.openclaw/workspace/memory/create-daily-note.sh

# Weekly on Sunday at 3 AM: Archive old notes
0 3 * * 0 find memory/daily -name "*.md" -mtime +30 -exec mv {} memory/archive/ \;
```

### Manual Commands

```bash
# Create today's note immediately
~/.openclaw/workspace/memory/create-daily-note.sh

# Search all memory
~/.openclaw/workspace/memory/memory-search.sh "query"

# List all daily notes
ls -la ~/.openclaw/workspace/memory/daily/
```

---

## 🎯 Best Practices

### For Daily Notes
- Update throughout the day
- Link to relevant files
- Note observations and insights
- Mark completed items with ✅

### For MEMORY.md
- Keep it curated (not everything)
- Update when important decisions are made
- Include user preferences
- Add quick links for fast access

### For Project Notes
- Include architecture diagrams
- Document key commands
- Track version history
- Link to external resources

---

## 🔗 Integration

This memory system integrates with OpenClaw:

- **Automatic loading:** MEMORY.md loads in main sessions
- **Daily context:** Today's note is checked each session
- **Search:** Built-in search across all memory
- **Sync:** Part of git repository (if committed)

---

## 📝 Maintenance

### Weekly (Sundays at 3 AM)
- Old daily notes (>30 days) moved to archive/
- Logs cleaned up

### Monthly
- Review MEMORY.md for outdated info
- Archive completed projects
- Clean up archive/ folder

### As Needed
- Update project notes when status changes
- Add new projects to projects/
- Curate important decisions into MEMORY.md

---

**Status:** ✅ Active  
**Created:** 2026-02-24  
**Last Updated:** 2026-02-24
