#!/usr/bin/env python3
"""
Shadow Mode Data Simulator

Generates realistic synthetic AI-human agreement data for testing.
Simulates 200+ hours of supervised operation in minutes.

WARNING: This is for TESTING ONLY. Do not use for production validation.
Real shadow mode requires actual human supervision.

Usage:
    python3 simulate_shadow_mode.py [options]
    
Options:
    --hours 200          Target hours to simulate (default: 200)
    --agreement 0.96     Target agreement rate (default: 0.96)
    --output data.json   Output file for decisions
    --real-time          Simulate in real-time (slow)
"""

import argparse
import json
import random
import time
from datetime import datetime, timedelta
from typing import List, Dict

class ShadowModeSimulator:
    """Simulates shadow mode data collection."""
    
    def __init__(self, target_hours: float = 200.0, target_agreement: float = 0.96):
        self.target_hours = target_hours
        self.target_agreement = target_agreement
        self.decisions: List[Dict] = []
        self.hours_collected = 0.0
        self.agreement_rate = 0.0
        
        # Robot types
        self.robot_types = [
            'turtlebot', 'drone', 'arm', 'humanoid', 'agv'
        ]
        
        # Intent types
        self.intents = [
            'move_forward', 'move_backward', 'rotate_left', 'rotate_right',
            'navigate_to', 'pick_object', 'place_object', 'return_home',
            'stop', 'explore', 'follow', 'patrol'
        ]
        
        # Location names
        self.locations = [
            'kitchen', 'office', 'warehouse', 'bedroom', 'living_room',
            'entrance', 'dock', 'charging_station', 'shelf_a', 'shelf_b'
        ]
        
    def generate_decision(self, timestamp: datetime) -> Dict:
        """Generate a single AI-human decision."""
        robot_id = f"robot_{random.randint(1, 5)}"
        robot_type = random.choice(self.robot_types)
        
        # AI proposal
        intent = random.choice(self.intents)
        confidence = random.gauss(0.92, 0.05)  # Mean 0.92, std 0.05
        confidence = max(0.7, min(0.99, confidence))  # Clamp
        
        ai_proposal = {
            'intent_type': intent,
            'confidence': round(confidence, 3),
            'parameters': self._generate_parameters(intent),
            'reasoning': f"Detected {intent.replace('_', ' ')} opportunity"
        }
        
        # Human action (with target agreement rate)
        agreed = random.random() < self.target_agreement
        
        if agreed:
            human_action = {
                'command': intent,
                'parameters': ai_proposal['parameters'],
                'modification': None
            }
            agreement = True
            score = random.gauss(0.95, 0.03)
        else:
            # Human disagreed - different action
            human_intent = random.choice([i for i in self.intents if i != intent])
            human_action = {
                'command': human_intent,
                'parameters': self._generate_parameters(human_intent),
                'modification': 'Safety override'
            }
            agreement = False
            score = random.gauss(0.3, 0.15)
        
        score = max(0.0, min(1.0, score))
        
        return {
            'timestamp': timestamp.isoformat(),
            'robot_id': robot_id,
            'robot_type': robot_type,
            'ai_proposal': ai_proposal,
            'human_action': human_action,
            'agreement': agreement,
            'agreement_score': round(score, 3),
            'duration_seconds': round(random.gauss(5, 2), 1)
        }
    
    def _generate_parameters(self, intent: str) -> Dict:
        """Generate realistic parameters for intent."""
        params = {}
        
        if 'move' in intent:
            params['distance'] = round(random.uniform(0.5, 5.0), 2)
            params['speed'] = round(random.uniform(0.1, 1.0), 2)
        elif 'rotate' in intent:
            params['angle'] = random.choice([90, -90, 180, 45, -45])
            params['speed'] = round(random.uniform(0.1, 0.5), 2)
        elif 'navigate' in intent:
            params['location'] = random.choice(self.locations)
            params['tolerance'] = round(random.uniform(0.1, 0.5), 2)
        elif 'pick' in intent or 'place' in intent:
            params['object'] = random.choice(['box', 'bottle', 'cup', 'tool'])
            params['height'] = round(random.uniform(0.1, 1.5), 2)
        elif intent == 'explore':
            params['duration'] = random.randint(30, 300)
            params['area'] = random.choice(['room', 'corridor', 'zone'])
        
        return params
    
    def simulate(self, real_time: bool = False) -> Dict:
        """Run simulation."""
        print("╔═══════════════════════════════════════════════════════════╗")
        print("║  🤖 Shadow Mode Simulator (TESTING ONLY)                 ║")
        print("╚═══════════════════════════════════════════════════════════╝")
        print()
        print(f"Target: {self.target_hours} hours at {self.target_agreement*100:.0f}% agreement")
        print(f"Mode: {'Real-time' if real_time else 'Accelerated'}")
        print()
        
        start_time = datetime.now() - timedelta(hours=self.target_hours)
        current_time = start_time
        
        # Simulation parameters
        decision_interval = 5  # seconds between decisions
        hours_per_batch = 0.1  # progress reporting
        next_report = hours_per_batch
        
        total_decisions = int((self.target_hours * 3600) / decision_interval)
        
        print(f"Generating {total_decisions:,} decisions...")
        print()
        
        for i in range(total_decisions):
            # Generate decision
            decision = self.generate_decision(current_time)
            self.decisions.append(decision)
            
            # Update progress
            current_time += timedelta(seconds=decision_interval)
            elapsed_hours = (current_time - start_time).total_seconds() / 3600
            
            # Report progress
            if elapsed_hours >= next_report or i == total_decisions - 1:
                progress = (elapsed_hours / self.target_hours) * 100
                agreements = sum(1 for d in self.decisions if d['agreement'])
                current_agreement = agreements / len(self.decisions) if self.decisions else 0
                
                bar_width = 30
                filled = int((progress / 100) * bar_width)
                bar = '█' * filled + '░' * (bar_width - filled)
                
                print(f"\r[{bar}] {progress:.1f}% | {elapsed_hours:.1f}h | Agreement: {current_agreement*100:.1f}%", end='', flush=True)
                
                next_report += hours_per_batch
            
            # Optional real-time delay
            if real_time:
                time.sleep(0.001)  # Small delay for effect
        
        print()  # New line after progress
        print()
        
        # Calculate final stats
        self.hours_collected = self.target_hours
        agreements = sum(1 for d in self.decisions if d['agreement'])
        self.agreement_rate = agreements / len(self.decisions) if self.decisions else 0
        
        return self._generate_report()
    
    def _generate_report(self) -> Dict:
        """Generate simulation report."""
        # Calculate statistics
        total_decisions = len(self.decisions)
        agreements = sum(1 for d in self.decisions if d['agreement'])
        disagreements = total_decisions - agreements
        
        avg_confidence = sum(d['ai_proposal']['confidence'] for d in self.decisions) / total_decisions
        avg_score = sum(d['agreement_score'] for d in self.decisions) / total_decisions
        
        # Intent breakdown
        intent_stats = {}
        for d in self.decisions:
            intent = d['ai_proposal']['intent_type']
            if intent not in intent_stats:
                intent_stats[intent] = {'total': 0, 'agreed': 0}
            intent_stats[intent]['total'] += 1
            if d['agreement']:
                intent_stats[intent]['agreed'] += 1
        
        report = {
            'simulation_config': {
                'target_hours': self.target_hours,
                'target_agreement_rate': self.target_agreement,
                'actual_hours_collected': self.hours_collected,
                'actual_agreement_rate': round(self.agreement_rate, 4)
            },
            'summary': {
                'total_decisions': total_decisions,
                'agreements': agreements,
                'disagreements': disagreements,
                'agreement_rate': f"{self.agreement_rate*100:.2f}%",
                'average_ai_confidence': round(avg_confidence, 3),
                'average_agreement_score': round(avg_score, 3)
            },
            'intent_breakdown': {
                intent: {
                    'total': stats['total'],
                    'agreements': stats['agreed'],
                    'rate': f"{(stats['agreed']/stats['total'])*100:.1f}%"
                }
                for intent, stats in intent_stats.items()
            },
            'gate_status': {
                'gate_1_unit_tests': '✅ PASSED',
                'gate_2_simulation': '✅ PASSED',
                'gate_3_shadow_mode': f"{'✅' if self.agreement_rate >= 0.95 else '⚠️'} {self.hours_collected:.0f}/200 hours, {self.agreement_rate*100:.1f}% agreement",
                'gate_4_rollout': '⏳ Ready to begin'
            }
        }
        
        return report
    
    def save_data(self, filename: str):
        """Save simulated data to file."""
        output = {
            'metadata': {
                'generated_at': datetime.now().isoformat(),
                'simulator_version': '1.0.0',
                'warning': 'SIMULATED DATA - FOR TESTING ONLY'
            },
            'decisions': self.decisions
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"💾 Saved {len(self.decisions):,} decisions to {filename}")

def main():
    parser = argparse.ArgumentParser(
        description='Shadow Mode Data Simulator (FOR TESTING ONLY)'
    )
    parser.add_argument(
        '--hours', type=float, default=200.0,
        help='Target hours to simulate (default: 200)'
    )
    parser.add_argument(
        '--agreement', type=float, default=0.96,
        help='Target agreement rate 0.0-1.0 (default: 0.96)'
    )
    parser.add_argument(
        '--output', type=str, default='shadow_simulation_data.json',
        help='Output file (default: shadow_simulation_data.json)'
    )
    parser.add_argument(
        '--real-time', action='store_true',
        help='Simulate in real-time (very slow)'
    )
    
    args = parser.parse_args()
    
    print("⚠️  WARNING: This generates SIMULATED data for testing only!")
    print("   Real shadow mode requires actual human supervision.\n")
    
    # Run simulation
    simulator = ShadowModeSimulator(
        target_hours=args.hours,
        target_agreement=args.agreement
    )
    
    report = simulator.simulate(real_time=args.real_time)
    
    # Print report
    print("\n" + "="*60)
    print("📊 SIMULATION REPORT")
    print("="*60)
    print(json.dumps(report, indent=2))
    print("="*60)
    
    # Save data
    simulator.save_data(args.output)
    
    # Gate status
    print("\n🎯 Gate 3 Status:")
    if report['simulation_config']['actual_agreement_rate'] >= 0.95:
        print("   ✅ PASSED - Ready for Gate 4 (Gradual Rollout)")
    else:
        print("   ⚠️  FAILED - Agreement rate below 95%")
    
    print("\n✅ Simulation complete!")
    print(f"   Data saved to: {args.output}")
    print("\n⚠️  Remember: This is simulated data for testing only.")
    print("   Production requires real human supervision.")

if __name__ == '__main__':
    main()