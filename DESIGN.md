# Design System: Agent ROS Bridge

## 1. Visual Theme & Atmosphere

Agent ROS Bridge is a safety-critical robotics gateway — the interface between AI intelligence and physical robots. The design system embodies this duality: the precision of industrial robotics meets the clarity of modern developer tools. The visual language communicates trust, safety, and technical sophistication.

The interface is intentionally dark-mode-first (`#0a0a0f` to `#121218`), reflecting the serious nature of robot control systems. This is not a casual consumer app; it's infrastructure for machines that move in the real world. The darkness serves functional purposes: reduced eye strain for operators monitoring systems for hours, and high-contrast visibility for status indicators that demand attention.

The color system is built around a safety-centric semantic palette. Greens indicate safe/autonomous states, ambers signal human-attention-required, and reds demand immediate intervention. These aren't decorative choices — they map directly to the safety state machine: `simulation_only` → `supervised` → `validated` → `autonomous`.

Typography uses Inter for UI clarity and JetBrains Mono for code, logs, and technical readouts. The monospace voice is essential — this is a system where operators read stack traces, ROS topic names, and safety validation logs. Line heights are generous (1.6) for readability during long monitoring sessions.

**Key Characteristics:**
- Safety-first semantic colors: Green (safe), Amber (attention), Red (emergency)
- Dark canvas with high-contrast data visualization
- Monospace dominance for technical content
- Status indicators as first-class UI elements
- Real-time telemetry visualization
- Human-in-the-loop approval workflows prominently displayed

## 2. Color Palette & Roles

### Background Surfaces
- **Deep Void** (`#0a0a0f`): Deepest background — terminal-like canvas for the main interface
- **Panel Dark** (`#121218`): Sidebar, navigation, panel backgrounds
- **Surface Elevated** (`#1a1a24`): Cards, modals, dropdowns, elevated content areas
- **Surface Hover** (`#222230`): Hover states, selected items
- **Input Background** (`#0f0f14`): Form fields, code blocks, terminal windows

### Safety Status Colors (Semantic)
- **Safe Green** (`#10b981`): Autonomous mode available, validation passed, all systems nominal
- **Safe Green Dim** (`#059669`): Background tint for safe state indicators
- **Attention Amber** (`#f59e0b`): Human approval required, supervised mode, pending validation
- **Attention Amber Dim** (`#d97706`): Background tint for attention states
- **Emergency Red** (`#ef4444`): Emergency stop active, safety violation, system fault
- **Emergency Red Dim** (`#dc2626`): Background tint for emergency states
- **Info Blue** (`#3b82f6`): Informational states, help, documentation links

### Text & Content
- **Text Primary** (`#f8fafc`): Headlines, primary content — near-white with cool undertone
- **Text Secondary** (`#94a3b8`): Body text, descriptions — readable gray
- **Text Tertiary** (`#64748b`): Metadata, timestamps, disabled states
- **Text Muted** (`#475569`): Subtle labels, hints, placeholders
- **Text Inverse** (`#0a0a0f`): Text on light/solid colored backgrounds

### Brand & Accent
- **Brand Indigo** (`#6366f1`): Primary brand color — CTAs, active navigation, key actions
- **Brand Indigo Hover** (`#818cf8`): Hover state for brand elements
- **Brand Violet** (`#8b5cf6`): Secondary accent — highlights, special features
- **Brand Cyan** (`#06b6d4`): Technical accent — code syntax, telemetry data

### Borders & Dividers
- **Border Subtle** (`#1e1e2e`): Default borders, panel divisions
- **Border Standard** (`#2d2d3d`): Card borders, input borders
- **Border Focus** (`#6366f1`): Focus rings, active states
- **Divider** (`#1e1e2e`): Section dividers, separator lines

### Telemetry & Data Viz
- **Telemetry Primary** (`#22d3ee`): Live data streams, active connections
- **Telemetry Secondary** (`#818cf8`): Secondary metrics, historical data
- **Chart Grid** (`#1e1e2e`): Chart gridlines, axes
- **Chart Line Safe** (`#10b981`): Positive trend lines
- **Chart Line Warning** (`#f59e0b`): Cautionary trends
- **Chart Line Critical** (`#ef4444`): Critical thresholds

## 3. Typography Rules

### Font Family
- **Primary**: `Inter`, with fallbacks: `system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, Roboto`
- **Monospace**: `JetBrains Mono`, with fallbacks: `ui-monospace, SF Mono, Menlo, Monaco, Consolas`
- **Technical/Logs**: `JetBrains Mono` exclusively — stack traces, ROS topics, safety logs

### Hierarchy

| Role | Font | Size | Weight | Line Height | Letter Spacing | Use Case |
|------|------|------|--------|-------------|----------------|----------|
| Display | Inter | 48px (3rem) | 700 | 1.1 | -0.02em | Hero headlines, landing pages |
| H1 | Inter | 32px (2rem) | 700 | 1.2 | -0.01em | Page titles |
| H2 | Inter | 24px (1.5rem) | 600 | 1.3 | -0.01em | Section headings |
| H3 | Inter | 20px (1.25rem) | 600 | 1.4 | 0 | Card titles, subsections |
| H4 | Inter | 18px (1.125rem) | 600 | 1.4 | 0 | Feature headings |
| Body Large | Inter | 18px (1.125rem) | 400 | 1.6 | 0 | Introductions, important body text |
| Body | Inter | 16px (1rem) | 400 | 1.6 | 0 | Standard body text |
| Body Small | Inter | 14px (0.875rem) | 400 | 1.5 | 0 | Secondary text, captions |
| Label | Inter | 12px (0.75rem) | 500 | 1.4 | 0.02em | Labels, badges, tags (uppercase optional) |
| Mono Body | JetBrains Mono | 14px (0.875rem) | 400 | 1.6 | 0 | Code blocks, logs |
| Mono Small | JetBrains Mono | 12px (0.75rem) | 400 | 1.5 | 0 | Inline code, metadata |
| Mono Label | JetBrains Mono | 11px (0.6875rem) | 500 | 1.4 | 0.02em | Technical labels, status codes |

### Principles
- **Monospace for technical content**: All robot IDs, ROS topics, command outputs use JetBrains Mono
- **Weight hierarchy**: 400 (body), 500 (labels/UI), 600 (headings), 700 (display)
- **Generous line-height**: 1.5–1.6 for readability during long monitoring sessions
- **Status colors in monospace**: Safety states rendered in colored monospace for scanability

## 4. Component Stylings

### Status Indicators (First-Class Citizens)

**Safety Status Badge**
- Padding: 6px 12px
- Radius: 6px
- Font: 12px JetBrains Mono, weight 500, uppercase
- States:
  - Safe: `bg-emerald-500/10 text-emerald-400 border-emerald-500/20`
  - Attention: `bg-amber-500/10 text-amber-400 border-amber-500/20`
  - Emergency: `bg-red-500/10 text-red-400 border-red-500/20`
- Border: 1px solid with 20% opacity

**Connection Status Dot**
- Size: 8px
- Shape: Circle (50% radius)
- States:
  - Connected: Solid emerald-500 with pulse animation
  - Connecting: Amber-500 with blink animation
  - Disconnected: Red-500 solid
  - Unknown: Slate-500 solid

**Telemetry Badge**
- Background: `bg-slate-800`
- Text: Monospace, cyan-400 for live data
- Format: `topic_name: value unit`
- Example: `/cmd_vel: 0.45 m/s`

### Buttons

**Primary Button**
- Background: `#6366f1` (brand indigo)
- Text: White, 14px weight 500
- Padding: 10px 20px
- Radius: 8px
- Hover: `#818cf8` with subtle lift shadow
- Focus: 2px ring in brand color

**Secondary Button**
- Background: `transparent`
- Border: 1px solid `#2d2d3d`
- Text: `#f8fafc`, 14px weight 500
- Padding: 10px 20px
- Radius: 8px
- Hover: Background `#1a1a24`

**Danger Button (Emergency Actions)**
- Background: `#dc2626`
- Text: White, 14px weight 600
- Padding: 10px 20px
- Radius: 8px
- Hover: `#b91c1c`
- Use: Emergency stop, safety overrides

**Ghost Button (Toolbar)**
- Background: `transparent`
- Text: `#94a3b8`, 14px
- Padding: 8px 12px
- Radius: 6px
- Hover: Background `#1a1a24`, text `#f8fafc`

### Cards

**Standard Card**
- Background: `#121218`
- Border: 1px solid `#1e1e2e`
- Radius: 12px
- Padding: 24px
- Shadow: None (flat design)

**Elevated Card (Modals, Overlays)**
- Background: `#1a1a24`
- Border: 1px solid `#2d2d3d`
- Radius: 16px
- Padding: 32px
- Shadow: `0 25px 50px -12px rgba(0, 0, 0, 0.5)`

**Status Card (Safety Dashboard)**
- Background: Contextual (green/amber/red tint at 5% opacity)
- Border: 1px solid matching color at 20% opacity
- Radius: 12px
- Header: Status icon + status text in monospace
- Content: Metrics, details

### Inputs

**Text Input**
- Background: `#0f0f14`
- Border: 1px solid `#1e1e2e`
- Text: `#f8fafc`
- Padding: 12px 16px
- Radius: 8px
- Focus: Border `#6366f1`, ring `#6366f1/20`
- Placeholder: `#475569`

**Command Input (Terminal-like)**
- Background: `#0a0a0f`
- Border: None
- Font: JetBrains Mono 14px
- Prompt: `>` in brand color
- Autocomplete: Dropdown with monospace suggestions

### Navigation

**Sidebar Navigation**
- Background: `#0a0a0f`
- Width: 240px
- Item padding: 12px 16px
- Active: Background `#1a1a24`, left border 3px brand color
- Icon + Label layout

**Top Navigation**
- Background: `#0a0a0f` with backdrop blur
- Height: 64px
- Border bottom: 1px solid `#1e1e2e`
- Logo left, status center, actions right

## 5. Layout Principles

### Spacing Scale
- **xs**: 4px
- **sm**: 8px
- **md**: 16px
- **lg**: 24px
- **xl**: 32px
- **2xl**: 48px
- **3xl**: 64px

### Grid
- 12-column grid system
- Gutter: 24px
- Max container width: 1440px
- Padding: 24px (desktop), 16px (mobile)

### Z-Index Layers
- **Base**: 0 — Content
- **Elevated**: 10 — Cards, panels
- **Sticky**: 100 — Sticky headers, sidebars
- **Overlay**: 1000 — Modals, dialogs
- **Critical**: 10000 — Emergency stop, toasts

### Safety Dashboard Layout
- **Left Sidebar**: Robot fleet status, connection health
- **Center**: Main content (3D view, logs, controls)
- **Right Panel**: Safety metrics, shadow mode stats, approvals
- **Bottom Bar**: Global safety status, emergency controls

## 6. Depth & Elevation

### Shadow System
- **Flat**: No shadow (default cards)
- **Subtle**: `0 1px 2px 0 rgba(0, 0, 0, 0.3)` — Buttons, inputs
- **Elevated**: `0 4px 6px -1px rgba(0, 0, 0, 0.4)` — Cards on hover
- **Floating**: `0 10px 15px -3px rgba(0, 0, 0, 0.5)` — Dropdowns, popovers
- **Modal**: `0 25px 50px -12px rgba(0, 0, 0, 0.6)` — Modals, dialogs

### Focus Rings
- Width: 2px
- Color: `#6366f1` (brand)
- Offset: 2px
- Style: Solid

## 7. Do's and Don'ts

### Do
- Use safety colors consistently (green = safe, amber = attention, red = emergency)
- Render robot IDs and technical identifiers in monospace
- Show real-time connection status prominently
- Use generous whitespace around safety-critical controls
- Provide immediate visual feedback for all actions
- Keep emergency stop visible and accessible at all times

### Don't
- Use red for non-emergency purposes
- Hide safety status behind menus or clicks
- Use decorative animations that could distract operators
- Render technical data in proportional fonts
- Use low contrast for status indicators
- Block the emergency stop with modals or overlays

## 8. Responsive Behavior

### Breakpoints
- **Mobile**: < 640px
- **Tablet**: 640px – 1024px
- **Desktop**: 1024px – 1440px
- **Wide**: > 1440px

### Mobile Adaptations
- Sidebar collapses to bottom sheet
- Safety status moves to persistent bottom bar
- Telemetry cards stack vertically
- Emergency stop becomes full-width floating button

### Touch Targets
- Minimum: 44px × 44px
- Emergency controls: 64px × 64px minimum
- Spacing between touch targets: 8px minimum

## 9. Agent Prompt Guide

### Quick Color Reference

**Status Colors:**
```
Safe:       #10b981 (emerald-500)
Attention:  #f59e0b (amber-500)  
Emergency:  #ef4444 (red-500)
Info:       #3b82f6 (blue-500)
```

**Brand Colors:**
```
Primary:    #6366f1 (indigo-500)
Secondary:  #8b5cf6 (violet-500)
Accent:     #06b6d4 (cyan-500)
```

**Backgrounds:**
```
Deepest:    #0a0a0f
Panel:      #121218
Card:       #1a1a24
Input:      #0f0f14
```

### Ready-to-Use Prompts

**"Create a safety status card"**
> Create a card component showing robot safety status. Use a 12px JetBrains Mono uppercase label with semantic color (emerald for safe, amber for attention, red for emergency). Include a connection status dot, robot ID in monospace, and key metrics. Background should use the status color at 5% opacity with a 20% opacity border.

**"Create an emergency stop button"**
> Create a prominent emergency stop button. Red background (#dc2626), white text, 64px touch target. Include a pulse animation when active. Position fixed, bottom-right on mobile, top-right on desktop. Should override all other UI elements with z-index 10000.

**"Create a telemetry panel"**
> Create a dark telemetry panel with topic names and values. Use JetBrains Mono for all text. Topics in slate-400, values in cyan-400 for live data. Include a small green pulsing dot next to live values. Background #0f0f14, border #1e1e2e.

**"Create a human approval workflow UI"**
> Create a modal for human-in-the-loop approval. Show AI proposal on left (indigo tint background), human controls on right. Include confidence score, proposed action details in monospace, and clear Approve/Reject buttons. Safety warning banner in amber.
