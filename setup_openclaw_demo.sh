#!/bin/bash
# Setup script for OpenClaw Agent ROS Bridge Demo

echo "🚀 Setting up OpenClaw Agent ROS Bridge Demo"
echo "=============================================="

# Check if we're in the right directory
if [ ! -f "agent_ros_bridge/__init__.py" ]; then
    echo "❌ Error: Not in Agent ROS Bridge root directory"
    echo "   Please run from: /Users/webthree/.openclaw/workspace"
    exit 1
fi

echo ""
echo "📦 Step 1: Verifying Agent ROS Bridge installation..."
python3 -c "from agent_ros_bridge import RobotAgent; print('✅ Agent ROS Bridge installed')" || {
    echo "❌ Installing Agent ROS Bridge..."
    pip install -e .
}

echo ""
echo "🔧 Step 2: Checking AI integrations..."
python3 -c "
import sys
sys.path.insert(0, 'examples/ai_agent_integrations')
from openclaw_integration import OpenClawROSBridge
print('✅ OpenClaw integration ready')
" || {
    echo "⚠️  Warning: Could not import OpenClaw integration"
    echo "   This is normal if running for the first time"
}

echo ""
echo "🎮 Step 3: Demo Options"
echo "=============================================="
echo ""
echo "Choose a demo to run:"
echo ""
echo "1) Interactive OpenClaw Demo (Recommended)"
echo "   Type natural language commands and see AI interpretation"
echo "   Command: python3 examples/ai_agent_integrations/openclaw_integration_demo.py"
echo ""
echo "2) Natural Language Examples"
echo "   See context-aware NL generation and interpretation"
echo "   Command: python3 examples/ai_agent_integrations/natural_language_examples.py"
echo ""
echo "3) Quick Test"
echo "   Verify everything is working"
echo "   Command: python3 -c \"from agent_ros_bridge.agentic import RobotAgent; r = RobotAgent(device_id='test'); print('✅ Working')\""
echo ""
echo "=============================================="
echo ""

# Ask user which demo to run
read -p "Which demo would you like to run? (1/2/3/q): " choice

case $choice in
    1)
        echo ""
        echo "🚀 Starting Interactive OpenClaw Demo..."
        echo "=============================================="
        python3 examples/ai_agent_integrations/openclaw_integration_demo.py
        ;;
    2)
        echo ""
        echo "🚀 Starting Natural Language Examples..."
        echo "=============================================="
        python3 examples/ai_agent_integrations/natural_language_examples.py
        ;;
    3)
        echo ""
        echo "🧪 Running Quick Test..."
        echo "=============================================="
        python3 -c "
from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.tools import ROSTopicEchoTool
from examples.ai_agent_integrations.openclaw_integration import OpenClawROSBridge

print('Testing RobotAgent...')
r = RobotAgent(device_id='test')
print(f'✅ RobotAgent created: {r.device_id}')

print('Testing OpenClaw Bridge...')
bridge = OpenClawROSBridge(r)
print('✅ OpenClawROSBridge created')

print('Testing Tool...')
tool = ROSTopicEchoTool()
print(f'✅ Tool created: {tool.name}')

print('')
print('🎉 All components working!')
print('Ready to run full demo.')
"
        ;;
    q|Q)
        echo ""
        echo "👋 Setup complete. Run demos anytime with:"
        echo "  python3 examples/ai_agent_integrations/openclaw_integration_demo.py"
        ;;
    *)
        echo ""
        echo "❌ Invalid choice. Setup complete."
        echo "Run demos manually:"
        echo "  python3 examples/ai_agent_integrations/openclaw_integration_demo.py"
        ;;
esac

echo ""
echo "=============================================="
echo "✅ Setup complete!"
echo ""
echo "Next steps:"
echo "  1. Run the interactive demo:"
echo "     python3 examples/ai_agent_integrations/openclaw_integration_demo.py"
echo ""
echo "  2. Try natural language commands like:"
echo "     - 'Go to the kitchen'"
echo "     - 'Check robot status'"
echo "     - 'What's the battery level?'"
echo ""
echo "  3. Read the guide:"
echo "     cat OPENCLAW_DEMO_GUIDE.md"
echo "=============================================="
