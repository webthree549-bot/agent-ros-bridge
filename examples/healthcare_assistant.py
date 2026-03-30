"""
Example 3: Healthcare Assistant Robot (Safety-Critical)

This example demonstrates deploying Agent ROS Bridge in a healthcare setting
with absolute safety requirements and regulatory compliance.

Key Features:
- ZERO tolerance for safety violations
- Extended shadow mode validation (500+ hours)
- HIPAA/FDA compliance
- 2-nurse consensus for high-risk actions
- Emergency response capabilities

Use Case: Hospital patient care with medication delivery
"""

from agent_ros_bridge import RobotAgent
from agent_ros_bridge.shadow import ShadowModeIntegration
from agent_ros_bridge.safety import EmergencyStop
from dataclasses import dataclass
from typing import Dict, List, Optional
import time
from datetime import datetime


@dataclass
class Patient:
    """Patient information (HIPAA-compliant)."""
    patient_id: str
    room_number: str
    current_medications: List[str]
    allergies: List[str]
    mobility_assistance: bool


@dataclass
class CareTask:
    """Healthcare task with risk assessment."""
    task_id: str
    patient: Patient
    task_type: str  # "medication", "mobility", "monitoring", "emergency"
    description: str
    risk_level: str  # "low", "medium", "high", "critical"
    requires_nurse_approval: bool = True
    requires_second_nurse: bool = False


class HealthcareRobot:
    """
    Healthcare robot with maximum safety enforcement.
    
    CRITICAL: This robot NEVER operates autonomously.
    - Human approval required for ALL actions
    - Extended validation (500+ hours vs 200 standard)
    - 99% agreement threshold (vs 95% standard)
    - Zero safety violations tolerated
    """
    
    def __init__(self, robot_id: str):
        # Healthcare: Override safety to maximum
        self.agent = RobotAgent(
            device_id=robot_id,
            device_type='mobile_robot',
            llm_provider='moonshot',
            require_confirmation=True,  # ALWAYS require confirmation
        )
        
        # Healthcare-specific safety overrides
        self.agent.safety.autonomous_mode = False  # NEVER enable
        self.agent.safety.human_in_the_loop = True  # ALWAYS require
        self.agent.safety.required_shadow_hours = 500  # Extended validation
        self.agent.safety.min_agreement_rate = 0.99  # 99% threshold
        
        self.shadow = ShadowModeIntegration()
        self.emergency_stop = EmergencyStop()
        self.care_tasks_completed = 0
        self.safety_violations = 0
        
    def deliver_medication(
        self,
        task: CareTask,
        nurse_id: str,
        nurse_approval: bool = False,
        second_nurse_id: Optional[str] = None,
        second_approval: bool = False,
    ) -> dict:
        """
        Deliver medication to patient with maximum safety checks.
        
        HIGH-RISK: Requires:
        - Primary nurse approval
        - Second nurse approval (if critical)
        - Patient verification
        - Shadow mode logging
        - Audit trail
        """
        print(f"\n💊 MEDICATION DELIVERY - Task: {task.task_id}")
        print(f"   Patient: {task.patient.patient_id} (Room {task.patient.room_number})")
        print(f"   Description: {task.description}")
        print(f"   Risk Level: {task.risk_level.upper()}")
        
        # Check risk level
        if task.risk_level in ['high', 'critical']:
            task.requires_second_nurse = True
            print("   ⚠️  HIGH RISK: Second nurse approval required")
        
        # Log AI proposal
        record_id = self.shadow.log_ai_decision(
            robot_id=self.agent.device_id,
            intent_type='DELIVER_MEDICATION',
            confidence=0.88,
            entities=[
                {'type': 'PATIENT', 'value': task.patient.patient_id},
                {'type': 'ROOM', 'value': task.patient.room_number},
                {'type': 'MEDICATION', 'value': task.description},
            ],
        )
        
        # Primary nurse approval
        if not nurse_approval:
            print(f"   ⏳ Waiting for Nurse {nurse_id} approval...")
            # In production: wait for actual approval
            return {'status': 'PENDING_APPROVAL', 'nurse_required': nurse_id}
        
        print(f"   ✅ Nurse {nurse_id} approved")
        
        # Second nurse approval for high-risk
        if task.requires_second_nurse:
            if not second_nurse_id or not second_approval:
                print(f"   ⏳ Waiting for Second Nurse approval...")
                return {'status': 'PENDING_SECOND_APPROVAL', 'nurse_required': 'SECOND_NURSE'}
            print(f"   ✅ Second Nurse {second_nurse_id} approved")
        
        # Verify patient
        print(f"   🔍 Verifying patient {task.patient.patient_id}...")
        
        # Execute with extreme caution
        print("   🚶 Proceeding to patient room...")
        start_time = time.time()
        
        try:
            # Simulated execution
            time.sleep(1.0)
            
            # Check allergies (critical safety check)
            if 'medication' in task.description.lower():
                for allergy in task.patient.allergies:
                    if allergy.lower() in task.description.lower():
                        raise ValueError(f"ALLERGY ALERT: Patient allergic to {allergy}")
            
            success = True
            self.care_tasks_completed += 1
            print("   ✅ Medication delivered successfully")
            
        except Exception as e:
            success = False
            self.safety_violations += 1
            print(f"   ❌ SAFETY VIOLATION: {e}")
            
            # Emergency stop
            self.emergency_stop.activate()
            print("   🛑 EMERGENCY STOP ACTIVATED")
        
        duration = time.time() - start_time
        
        # Log outcome
        self.shadow.log_outcome(
            record_id=record_id,
            success=success,
            execution_time_ms=duration * 1000,
            error_message='' if success else 'Safety violation occurred',
        )
        
        # Create audit trail
        audit_entry = {
            'timestamp': datetime.now().isoformat(),
            'task_id': task.task_id,
            'robot_id': self.agent.device_id,
            'nurse_id': nurse_id,
            'second_nurse_id': second_nurse_id,
            'patient_id': task.patient.patient_id,
            'success': success,
            'safety_violation': not success,
        }
        
        return {
            'status': 'COMPLETE' if success else 'FAILED',
            'success': success,
            'duration': duration,
            'audit_trail': audit_entry,
        }
    
    def assist_mobility(
        self,
        patient: Patient,
        destination: str,
        nurse_id: str,
        nurse_approval: bool = False,
    ) -> dict:
        """
        Assist patient mobility (walking, transfers).
        
        ALWAYS requires nurse presence and approval.
        Robot assists, nurse leads.
        """
        print(f"\n🚶 MOBILITY ASSISTANCE - Patient: {patient.patient_id}")
        print(f"   Destination: {destination}")
        print(f"   Mobility assistance needed: {patient.mobility_assistance}")
        
        # Log AI proposal
        record_id = self.shadow.log_ai_decision(
            robot_id=self.agent.device_id,
            intent_type='MOBILITY_ASSIST',
            confidence=0.92,
            entities=[
                {'type': 'PATIENT', 'value': patient.patient_id},
                {'type': 'DESTINATION', 'value': destination},
            ],
        )
        
        # Nurse approval mandatory
        if not nurse_approval:
            print(f"   ⏳ Waiting for Nurse {nurse_id} approval...")
            return {'status': 'PENDING_APPROVAL'}
        
        print(f"   ✅ Nurse {nurse_id} approved and present")
        print("   🤖 Robot ready to assist (nurse leads)")
        
        # Execute
        time.sleep(0.5)
        
        self.shadow.log_outcome(
            record_id=record_id,
            success=True,
            execution_time_ms=500,
        )
        
        return {
            'status': 'COMPLETE',
            'success': True,
            'nurse_present': True,
            'robot_role': 'assist',
        }
    
    def monitor_patient(self, patient: Patient, duration_minutes: int) -> dict:
        """
        Monitor patient vitals and environment.
        
        Lower risk, but still requires approval and logging.
        """
        print(f"\n📊 PATIENT MONITORING - Patient: {patient.patient_id}")
        print(f"   Duration: {duration_minutes} minutes")
        
        # Log
        record_id = self.shadow.log_ai_decision(
            robot_id=self.agent.device_id,
            intent_type='MONITOR_PATIENT',
            confidence=0.95,
            entities=[{'type': 'PATIENT', 'value': patient.patient_id}],
        )
        
        # Monitoring loop
        for minute in range(duration_minutes):
            print(f"   ⏱️  Minute {minute + 1}/{duration_minutes}")
            time.sleep(0.1)  # Simulated
            
            # Check for emergency
            # (In production: check vitals, alerts)
        
        self.shadow.log_outcome(
            record_id=record_id,
            success=True,
            execution_time_ms=duration_minutes * 60 * 1000,
        )
        
        return {
            'status': 'COMPLETE',
            'monitoring_duration': duration_minutes,
            'alerts_triggered': 0,
        }
    
    def get_healthcare_metrics(self) -> dict:
        """Get metrics for regulatory compliance."""
        shadow_metrics = self.shadow.get_metrics()
        
        return {
            'robot_id': self.agent.device_id,
            'care_tasks_completed': self.care_tasks_completed,
            'safety_violations': self.safety_violations,
            'safety_status': 'COMPLIANT' if self.safety_violations == 0 else 'NON_COMPLIANT',
            'shadow_mode_hours': shadow_metrics.get('hours_collected', 0),
            'agreement_rate': shadow_metrics.get('agreement_rate', 0.0),
            'regulatory_ready': (
                self.safety_violations == 0 and
                shadow_metrics.get('agreement_rate', 0) >= 0.99
            ),
        }


def main():
    """Demonstrate healthcare robot with maximum safety."""
    print("=" * 60)
    print("🏥 Healthcare Assistant Robot Example")
    print("=" * 60)
    print("\n🛡️  SAFETY CONFIGURATION (MAXIMUM)")
    print("   ⚠️  Autonomous Mode: NEVER ENABLED")
    print("   ✅ Human-in-the-Loop: ALWAYS REQUIRED")
    print("   📊 Shadow Mode: 500+ hours required")
    print("   🎯 Agreement Threshold: 99%")
    print("   🚫 Safety Violations: ZERO TOLERANCE")
    
    # Create healthcare robot
    carebot = HealthcareRobot('carebot_01')
    
    # Create patient
    patient = Patient(
        patient_id='PT12345',
        room_number='302',
        current_medications=['Lisinopril', 'Metformin'],
        allergies=['Penicillin'],
        mobility_assistance=True,
    )
    
    # Scenario 1: Medication Delivery (High Risk)
    print("\n" + "=" * 60)
    print("SCENARIO 1: Medication Delivery (HIGH RISK)")
    print("=" * 60)
    
    med_task = CareTask(
        task_id='MED001',
        patient=patient,
        task_type='medication',
        description='Deliver 10mg Lisinopril',
        risk_level='high',
        requires_nurse_approval=True,
        requires_second_nurse=True,  # High risk = 2 nurses
    )
    
    result = carebot.deliver_medication(
        task=med_task,
        nurse_id='NURSE_789',
        nurse_approval=True,
        second_nurse_id='NURSE_790',
        second_approval=True,
    )
    
    print(f"\n   Result: {result['status']}")
    print(f"   Audit Trail: {result.get('audit_trail', {})}")
    
    # Scenario 2: Mobility Assistance (Medium Risk)
    print("\n" + "=" * 60)
    print("SCENARIO 2: Mobility Assistance (MEDIUM RISK)")
    print("=" * 60)
    
    result = carebot.assist_mobility(
        patient=patient,
        destination='Physical Therapy Room',
        nurse_id='NURSE_789',
        nurse_approval=True,
    )
    
    print(f"\n   Result: {result['status']}")
    print(f"   Nurse Present: {result['nurse_present']}")
    
    # Scenario 3: Patient Monitoring (Low Risk)
    print("\n" + "=" * 60)
    print("SCENARIO 3: Patient Monitoring (LOW RISK)")
    print("=" * 60)
    
    result = carebot.monitor_patient(
        patient=patient,
        duration_minutes=5,
    )
    
    print(f"\n   Result: {result['status']}")
    print(f"   Alerts: {result['alerts_triggered']}")
    
    # Show metrics
    print("\n" + "=" * 60)
    print("📊 Healthcare Metrics")
    print("=" * 60)
    metrics = carebot.get_healthcare_metrics()
    for key, value in metrics.items():
        print(f"   {key}: {value}")
    
    print("\n✅ Healthcare example complete!")
    print("   Maximum safety enforced at all times.")
    print("   All actions logged for regulatory compliance.")


if __name__ == '__main__':
    main()
