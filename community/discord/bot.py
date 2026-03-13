"""Discord bot for Agent ROS Bridge community.

Provides support, status updates, and community features.
"""

import os

import discord
from discord.ext import commands, tasks


class AgentROSBot(commands.Bot):
    """Discord bot for Agent ROS Bridge community."""

    def __init__(self):
        intents = discord.Intents.default()
        intents.message_content = True
        intents.members = True

        super().__init__(
            command_prefix="!",
            intents=intents,
            help_command=commands.DefaultHelpCommand()
        )

        self.bridge_status = {"online": False, "robots": 0}

    async def setup_hook(self):
        """Setup bot."""
        # Start background tasks
        self.status_update.start()

    async def on_ready(self):
        """Called when bot is ready."""
        print(f"✅ Logged in as {self.user} (ID: {self.user.id})")
        print("-" * 40)

        # Set presence
        await self.change_presence(
            activity=discord.Activity(
                type=discord.ActivityType.watching,
                name="ROS robots | !help"
            )
        )

    async def on_command_error(self, ctx, error):
        """Handle command errors."""
        if isinstance(error, commands.CommandNotFound):
            return

        embed = discord.Embed(
            title="❌ Error",
            description=str(error),
            color=discord.Color.red()
        )
        await ctx.send(embed=embed)

    @tasks.loop(minutes=5)
    async def status_update(self):
        """Update status every 5 minutes."""
        # In production, this would check actual bridge status
        self.bridge_status["online"] = True
        self.bridge_status["robots"] = 3  # Mock data

    @commands.command(name="status")
    async def status(self, ctx):
        """Check Agent ROS Bridge status."""
        embed = discord.Embed(
            title="🤖 Agent ROS Bridge Status",
            color=discord.Color.green() if self.bridge_status["online"] else discord.Color.red()
        )

        embed.add_field(
            name="Status",
            value="🟢 Online" if self.bridge_status["online"] else "🔴 Offline",
            inline=True
        )
        embed.add_field(
            name="Connected Robots",
            value=str(self.bridge_status["robots"]),
            inline=True
        )
        embed.add_field(
            name="Version",
            value="0.5.0",
            inline=True
        )

        await ctx.send(embed=embed)

    @commands.command(name="docs")
    async def docs(self, ctx):
        """Get documentation links."""
        embed = discord.Embed(
            title="📚 Documentation",
            description="Official Agent ROS Bridge resources",
            color=discord.Color.blue()
        )

        embed.add_field(
            name="Getting Started",
            value="[Quick Start Guide](https://github.com/webthree549-bot/agent-ros-bridge#quick-start)",
            inline=False
        )
        embed.add_field(
            name="API Reference",
            value="[Full Documentation](https://docs.agentrosbridge.io)",
            inline=False
        )
        embed.add_field(
            name="Examples",
            value="[GitHub Examples](https://github.com/webthree549-bot/agent-ros-bridge/tree/main/examples)",
            inline=False
        )
        embed.add_field(
            name="Troubleshooting",
            value="[Common Issues](https://github.com/webthree549-bot/agent-ros-bridge/blob/main/docs/TROUBLESHOOTING.md)",
            inline=False
        )

        await ctx.send(embed=embed)

    @commands.command(name="install")
    async def install(self, ctx):
        """Show installation instructions."""
        embed = discord.Embed(
            title="🚀 Installation",
            description="Choose your preferred method:",
            color=discord.Color.blue()
        )

        methods = {
            "pip": "```bash\npip install agent-ros-bridge\n```",
            "Docker": "```bash\ndocker pull agentrosbridge/agent-ros-bridge\n```",
            "Helm": "```bash\nhelm install agent-ros-bridge ./helm/agent-ros-bridge\n```",
            "ClawHub": "```bash\nnpx clawhub install agent-ros-bridge\n```",
        }

        for name, code in methods.items():
            embed.add_field(name=name, value=code, inline=False)

        await ctx.send(embed=embed)

    @commands.command(name="support")
    async def support(self, ctx):
        """Get support information."""
        embed = discord.Embed(
            title="🆘 Support",
            description="Need help? Here are your options:",
            color=discord.Color.orange()
        )

        embed.add_field(
            name="GitHub Issues",
            value="[Report bugs](https://github.com/webthree549-bot/agent-ros-bridge/issues)",
            inline=False
        )
        embed.add_field(
            name="Documentation",
            value="[Troubleshooting Guide](https://github.com/webthree549-bot/agent-ros-bridge/blob/main/docs/TROUBLESHOOTING.md)",
            inline=False
        )
        embed.add_field(
            name="Community",
            value="Ask in #general or #support channels",
            inline=False
        )

        await ctx.send(embed=embed)

    @commands.command(name="example")
    async def example(self, ctx, example_type: str = "basic"):
        """Show code examples.
        
        Usage: !example [basic|langchain|mcp]
        """
        examples = {
            "basic": """```python
from agent_ros_bridge import AgentROSBridge

bridge = AgentROSBridge()
await bridge.start()

# Send natural language command
result = await bridge.execute_nl("move forward 1 meter")
print(result)
```""",
            "langchain": """```python
from agent_ros_bridge.frameworks.langchain import (
    AgentROSBridgeClient, get_ros_tools
)
from langchain.agents import initialize_agent

client = AgentROSBridgeClient()
tools = get_ros_tools(client)

agent = initialize_agent(tools, llm, agent="zero-shot-react-description")
agent.run("move the robot to the kitchen")
```""",
            "mcp": """```json
{
  "mcpServers": {
    "agent-ros": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.frameworks.mcp.server"]
    }
  }
}
```""",
        }

        code = examples.get(example_type, examples["basic"])

        embed = discord.Embed(
            title=f"💻 {example_type.title()} Example",
            description=code,
            color=discord.Color.purple()
        )

        await ctx.send(embed=embed)

    @commands.command(name="stats")
    async def stats(self, ctx):
        """Show community statistics."""
        embed = discord.Embed(
            title="📊 Community Stats",
            color=discord.Color.blue()
        )

        # Mock statistics - in production, fetch from database
        embed.add_field(name="GitHub Stars", value="⭐ 1.2k", inline=True)
        embed.add_field(name="Discord Members", value=f"👥 {ctx.guild.member_count}", inline=True)
        embed.add_field(name="PyPI Downloads", value="📦 50k/month", inline=True)

        await ctx.send(embed=embed)


def main():
    """Run the Discord bot."""
    token = os.getenv("DISCORD_BOT_TOKEN")

    if not token:
        print("❌ DISCORD_BOT_TOKEN environment variable not set")
        print("Set it with: export DISCORD_BOT_TOKEN=your_token")
        return

    bot = AgentROSBot()
    bot.run(token)


if __name__ == "__main__":
    main()
