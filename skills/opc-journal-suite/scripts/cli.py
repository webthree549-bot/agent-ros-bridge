#!/usr/bin/env python3
"""OPC Journal Suite - Command line entry point for /opc-journal command.

This script handles the user-invocable /opc-journal command,
parsing arguments and routing to appropriate handlers.
"""
import sys
import json
import argparse
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from coordinate import main as coordinate, detect_intent, get_skill_for_intent
except ImportError:
    from scripts.coordinate import main as coordinate, detect_intent, get_skill_for_intent


def parse_command_args(args_list):
    """Parse command line arguments for /opc-journal command."""
    parser = argparse.ArgumentParser(
        prog='opc-journal',
        description='OPC Journal Suite - Personal growth tracking and insight generation'
    )
    
    parser.add_argument(
        'action',
        choices=['init', 'record', 'search', 'export', 'analyze', 'milestone', 'task', 'insight', 'cron'],
        help='Action to perform'
    )
    
    parser.add_argument(
        'content',
        nargs='?',
        help='Content for record/task actions'
    )
    
    parser.add_argument(
        '--customer-id',
        default='OPC-001',
        help='Customer identifier (default: OPC-001)'
    )
    
    parser.add_argument(
        '--day',
        type=int,
        help='Day number in journey'
    )
    
    parser.add_argument(
        '--format',
        choices=['yaml', 'markdown', 'json'],
        default='yaml',
        help='Output format (default: yaml)'
    )
    
    parser.add_argument(
        '--topic',
        help='Topic for search/analyze actions'
    )
    
    parser.add_argument(
        '--dimension',
        choices=['work_hours', 'decision_style', 'stress_triggers', 'emotional', 'all'],
        default='all',
        help='Analysis dimension'
    )
    
    parser.add_argument(
        '--days',
        type=int,
        default=30,
        help='Number of days to analyze (default: 30)'
    )
    
    parser.add_argument(
        '--period',
        choices=['daily', 'weekly', 'monthly'],
        default='daily',
        help='Export period'
    )
    
    parser.add_argument(
        '--due',
        help='Due date for tasks (e.g., "tomorrow", "2026-04-01")'
    )
    
    parser.add_argument(
        '--cron-action',
        choices=['check', 'get-schedule', 'update-schedule'],
        default='check',
        help='Cron scheduler action'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Preview without executing'
    )
    
    return parser.parse_args(args_list)


def build_intent_from_args(args):
    """Build intent and parameters from parsed arguments."""
    intent_map = {
        'init': 'journal_init',
        'record': 'journal_record',
        'search': 'journal_search',
        'export': 'journal_export',
        'analyze': 'pattern_analyze',
        'milestone': 'milestone_detect',
        'task': 'task_create',
        'insight': 'insight_generate',
        'cron': 'cron_schedule',
    }
    
    intent = intent_map.get(args.action, 'unknown')
    
    # Build parameters based on action
    params = {
        'customer_id': args.customer_id,
        'format': args.format,
    }
    
    if args.day:
        params['day'] = args.day
    
    if args.action == 'record' and args.content:
        params['content'] = args.content
    elif args.action == 'task' and args.content:
        params['description'] = args.content
        if args.due:
            params['due'] = args.due
    elif args.action == 'search' and args.topic:
        params['topic'] = args.topic
    elif args.action == 'analyze':
        params['dimension'] = args.dimension
        params['days'] = args.days
    elif args.action == 'export':
        params['period'] = args.period
    elif args.action == 'cron':
        params['cron_action'] = args.cron_action
    
    if args.dry_run:
        params['dry_run'] = True
    
    return intent, params


def main():
    """Main entry point for /opc-journal command."""
    try:
        args = parse_command_args(sys.argv[1:])
    except SystemExit as e:
        # argparse exits on error/help
        return e.code if isinstance(e.code, int) else 1
    
    # Build intent and parameters
    intent, params = build_intent_from_args(args)
    
    # Build context for coordinate
    context = {
        'customer_id': args.customer_id,
        'input': params.get('content', ''),
        'intent': intent,
        'params': params,
    }
    
    if args.dry_run:
        # Preview mode
        target_skill = get_skill_for_intent(intent)
        print(json.dumps({
            'preview': True,
            'intent': intent,
            'target_skill': target_skill,
            'parameters': params,
            'note': 'Dry run - no changes made'
        }, indent=2, ensure_ascii=False))
        return 0
    
    # Execute via coordinate
    try:
        result = coordinate(context)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result.get('status') == 'success' else 1
    except Exception as e:
        print(json.dumps({
            'status': 'error',
            'error': str(e),
            'intent': intent,
            'params': params
        }, indent=2, ensure_ascii=False))
        return 1


if __name__ == '__main__':
    sys.exit(main())
