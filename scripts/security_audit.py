#!/usr/bin/env python3
"""Security audit script for Agent ROS Bridge.

Performs comprehensive security checks.
"""

import os
import sys
import subprocess
import json
from pathlib import Path
from typing import List, Dict, Any


class SecurityAuditor:
    """Performs security audits on the codebase."""
    
    def __init__(self, project_root: str = "."):
        self.root = Path(project_root)
        self.issues: List[Dict[str, Any]] = []
        self.warnings: List[Dict[str, Any]] = []
    
    def run_all_checks(self) -> bool:
        """Run all security checks.
        
        Returns:
            True if no critical issues found
        """
        print("🔒 Agent ROS Bridge Security Audit")
        print("=" * 50)
        
        checks = [
            ("Dependencies", self.check_dependencies),
            ("Secrets", self.check_secrets),
            ("File Permissions", self.check_file_permissions),
            ("Configuration", self.check_configuration),
            ("Code Patterns", self.check_code_patterns),
            ("Docker Security", self.check_docker_security),
            ("TLS/SSL", self.check_tls_configuration),
        ]
        
        for name, check_func in checks:
            print(f"\n📋 {name}...")
            try:
                check_func()
                print(f"   ✅ {name} check passed")
            except Exception as e:
                print(f"   ❌ {name} check failed: {e}")
                self.issues.append({
                    "check": name,
                    "error": str(e),
                    "severity": "high"
                })
        
        return self.report()
    
    def check_dependencies(self) -> None:
        """Check for vulnerable dependencies."""
        # Run safety check
        result = subprocess.run(
            ["safety", "check", "--json"],
            capture_output=True,
            text=True,
            cwd=self.root
        )
        
        if result.returncode != 0:
            try:
                vulnerabilities = json.loads(result.stdout)
                for vuln in vulnerabilities:
                    self.issues.append({
                        "check": "Dependencies",
                        "package": vuln.get("package_name"),
                        "vulnerability": vuln.get("vulnerability_id"),
                        "severity": vuln.get("severity", "medium")
                    })
            except json.JSONDecodeError:
                pass
    
    def check_secrets(self) -> None:
        """Check for hardcoded secrets."""
        secret_patterns = [
            (r'password\s*=\s*["\'][^"\']+["\']', "Hardcoded password"),
            (r'secret\s*=\s*["\'][^"\']+["\']', "Hardcoded secret"),
            (r'api_key\s*=\s*["\'][^"\']+["\']', "Hardcoded API key"),
            (r'token\s*=\s*["\'][^"\']{20,}["\']', "Hardcoded token"),
            (r'-----BEGIN (RSA |DSA |EC |OPENSSH )?PRIVATE KEY-----', "Private key"),
        ]
        
        import re
        
        for pattern, description in secret_patterns:
            for file_path in self.root.rglob("*.py"):
                if ".venv" in str(file_path) or "__pycache__" in str(file_path):
                    continue
                
                try:
                    content = file_path.read_text()
                    matches = re.finditer(pattern, content, re.IGNORECASE)
                    for match in matches:
                        # Skip test files and examples
                        if "test" in str(file_path) or "example" in str(file_path):
                            continue
                        
                        self.issues.append({
                            "check": "Secrets",
                            "file": str(file_path),
                            "line": content[:match.start()].count('\n') + 1,
                            "description": description,
                            "severity": "critical"
                        })
                except Exception:
                    pass
    
    def check_file_permissions(self) -> None:
        """Check for overly permissive file permissions."""
        sensitive_files = [
            self.root / "config" / "bridge.yaml",
            self.root / ".env",
            self.root / ".jwt_secret",
        ]
        
        for file_path in sensitive_files:
            if file_path.exists():
                stat = file_path.stat()
                mode = oct(stat.st_mode)[-3:]
                
                if int(mode) > 600:
                    self.warnings.append({
                        "check": "File Permissions",
                        "file": str(file_path),
                        "mode": mode,
                        "recommended": "600",
                        "severity": "medium"
                    })
    
    def check_configuration(self) -> None:
        """Check configuration for security issues."""
        config_file = self.root / "config" / "bridge.yaml"
        
        if config_file.exists():
            content = config_file.read_text()
            
            # Check for debug mode in production
            if "debug: true" in content.lower():
                self.warnings.append({
                    "check": "Configuration",
                    "issue": "Debug mode enabled",
                    "recommendation": "Set debug: false for production",
                    "severity": "medium"
                })
            
            # Check for weak JWT secret
            if "jwt_secret:" in content:
                import re
                match = re.search(r'jwt_secret:\s*["\']?([^"\'\n]+)', content)
                if match:
                    secret = match.group(1)
                    # Skip if null or empty (will be loaded from env)
                    if secret in ("null", "", "~"):
                        self.warnings.append({
                            "check": "Configuration",
                            "issue": "JWT secret not set in config",
                            "recommendation": "Set JWT_SECRET environment variable for production",
                            "severity": "medium"
                        })
                    elif len(secret) < 32:
                        self.issues.append({
                            "check": "Configuration",
                            "issue": "Weak JWT secret",
                            "recommendation": "Use at least 256-bit secret",
                            "severity": "high"
                        })
    
    def check_code_patterns(self) -> None:
        """Check for dangerous code patterns."""
        dangerous_patterns = [
            (r'eval\s*\(', "Use of eval()"),
            (r'exec\s*\(', "Use of exec()"),
            (r'subprocess\.call\s*\([^)]*shell\s*=\s*True', "Shell=True in subprocess"),
            (r'\.format\s*\([^)]*%', "Potential format string vulnerability"),
            (r'input\s*\(', "Use of input()"),
            (r'__import__\s*\(', "Dynamic import"),
        ]
        
        import re
        
        for pattern, description in dangerous_patterns:
            for file_path in self.root.rglob("*.py"):
                if ".venv" in str(file_path) or "__pycache__" in str(file_path):
                    continue
                
                try:
                    content = file_path.read_text()
                    matches = re.finditer(pattern, content)
                    for match in matches:
                        self.warnings.append({
                            "check": "Code Patterns",
                            "file": str(file_path),
                            "line": content[:match.start()].count('\n') + 1,
                            "description": description,
                            "severity": "medium"
                        })
                except Exception:
                    pass
    
    def check_docker_security(self) -> None:
        """Check Docker security best practices."""
        dockerfile = self.root / "Dockerfile"
        
        if dockerfile.exists():
            content = dockerfile.read_text()
            
            # Check for non-root user
            if "USER" not in content:
                self.issues.append({
                    "check": "Docker Security",
                    "issue": "No non-root user specified",
                    "recommendation": "Add 'USER <username>' directive",
                    "severity": "high"
                })
            
            # Check for health check
            if "HEALTHCHECK" not in content:
                self.warnings.append({
                    "check": "Docker Security",
                    "issue": "No HEALTHCHECK defined",
                    "recommendation": "Add HEALTHCHECK directive",
                    "severity": "low"
                })
    
    def check_tls_configuration(self) -> None:
        """Check TLS/SSL configuration."""
        # Check for TLS settings in config
        config_file = self.root / "config" / "bridge.yaml"
        
        if config_file.exists():
            content = config_file.read_text()
            
            if "tls:" not in content.lower() and "ssl:" not in content.lower():
                self.warnings.append({
                    "check": "TLS/SSL",
                    "issue": "No TLS configuration found",
                    "recommendation": "Enable TLS for production",
                    "severity": "medium"
                })
    
    def report(self) -> bool:
        """Generate and print audit report.
        
        Returns:
            True if no critical issues
        """
        print("\n" + "=" * 50)
        print("📊 Security Audit Report")
        print("=" * 50)
        
        # Critical issues
        critical = [i for i in self.issues if i.get("severity") == "critical"]
        high = [i for i in self.issues if i.get("severity") == "high"]
        medium = [i for i in self.issues if i.get("severity") == "medium"]
        
        print(f"\n🔴 Critical Issues: {len(critical)}")
        for issue in critical:
            print(f"   - {issue['check']}: {issue.get('description', issue.get('issue', 'Unknown'))}")
        
        print(f"\n🟠 High Severity: {len(high)}")
        for issue in high:
            print(f"   - {issue['check']}: {issue.get('description', issue.get('issue', 'Unknown'))}")
        
        print(f"\n🟡 Medium Severity: {len(medium)}")
        for issue in medium:
            print(f"   - {issue['check']}: {issue.get('description', issue.get('issue', 'Unknown'))}")
        
        print(f"\n🟢 Warnings: {len(self.warnings)}")
        
        # Summary
        print("\n" + "=" * 50)
        if critical or high:
            print("❌ Security audit FAILED")
            print("   Critical or high severity issues found.")
            print("   Please fix issues before deploying to production.")
            return False
        elif medium:
            print("⚠️  Security audit PASSED with warnings")
            print("   Medium severity issues should be addressed.")
            return True
        else:
            print("✅ Security audit PASSED")
            return True


def main():
    """Main entry point."""
    auditor = SecurityAuditor()
    success = auditor.run_all_checks()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
