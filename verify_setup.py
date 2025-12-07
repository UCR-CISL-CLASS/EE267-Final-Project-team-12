"""
Setup Verification Script
Checks if all dependencies and CARLA connection are working
"""

import sys
import os

def check_python_version():
    """Check if Python version is compatible."""
    print("Checking Python version...")
    version = sys.version_info
    if version.major == 3 and version.minor >= 7:
        print(f"✓ Python {version.major}.{version.minor}.{version.micro} - OK")
        return True
    else:
        print(f"✗ Python {version.major}.{version.minor} - Please use Python 3.7+")
        return False

def check_dependencies():
    """Check if required Python packages are installed."""
    print("\nChecking Python dependencies...")
    dependencies = {
        'numpy': 'NumPy',
        'matplotlib': 'Matplotlib'
    }
    
    all_installed = True
    for module, name in dependencies.items():
        try:
            __import__(module)
            print(f"✓ {name} - Installed")
        except ImportError:
            print(f"✗ {name} - NOT INSTALLED")
            all_installed = False
    
    if not all_installed:
        print("\nInstall missing packages with:")
        print("  pip install -r requirements.txt")
    
    return all_installed

def check_carla():
    """Check if CARLA Python API is accessible."""
    print("\nChecking CARLA Python API...")
    try:
        import carla
        print(f"✓ CARLA Python API - Found")
        return True
    except ImportError:
        print("✗ CARLA Python API - NOT FOUND")
        print("\nTroubleshooting:")
        print("1. Make sure CARLA is installed")
        print("2. The script will try to auto-detect CARLA's egg file")
        print("3. Or manually add CARLA to Python path:")
        print("   sys.path.append('/path/to/CARLA/PythonAPI/carla/dist/carla-*.egg')")
        return False

def check_carla_connection(host='localhost', port=2000):
    """Check if CARLA server is running and accessible."""
    print("\nChecking CARLA server connection...")
    try:
        import carla
        client = carla.Client(host, port)
        client.set_timeout(5.0)
        version = client.get_server_version()
        print(f"✓ CARLA Server - Connected (version {version})")
        return True
    except Exception as e:
        print(f"✗ CARLA Server - NOT CONNECTED")
        print(f"  Error: {e}")
        print("\nMake sure CARLA is running:")
        print("  cd /path/to/CARLA")
        print("  ./CarlaUE4.sh  # Linux")
        print("  # or")
        print("  CarlaUE4.exe   # Windows")
        return False

def check_project_files():
    """Check if all required project files exist."""
    print("\nChecking project files...")
    required_files = [
        'pure_pursuit.py',
        'stanley.py',
        'experiment_runner.py',
        'evaluate_results.py',
        'README.md'
    ]
    
    all_present = True
    for filename in required_files:
        if os.path.exists(filename):
            print(f"✓ {filename} - Found")
        else:
            print(f"✗ {filename} - MISSING")
            all_present = False
    
    return all_present

def main():
    """Run all verification checks."""
    print("="*60)
    print("CARLA Lane Keeping Project - Setup Verification")
    print("="*60)
    
    checks = {
        'Python Version': check_python_version(),
        'Dependencies': check_dependencies(),
        'Project Files': check_project_files(),
        'CARLA API': check_carla(),
    }
    
    # Only check server connection if API is available
    if checks['CARLA API']:
        checks['CARLA Server'] = check_carla_connection()
    else:
        checks['CARLA Server'] = False
    
    print("\n" + "="*60)
    print("VERIFICATION SUMMARY")
    print("="*60)
    
    all_passed = True
    for check_name, passed in checks.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{check_name:.<40} {status}")
        if not passed:
            all_passed = False
    
    print("="*60)
    
    if all_passed:
        print("\n🎉 All checks passed! You're ready to run experiments.")
        print("\nNext steps:")
        print("  1. Make sure CARLA is running")
        print("  2. Run: python experiment_runner.py")
        print("  3. After completion, run: python evaluate_results.py")
    else:
        print("\n⚠️  Some checks failed. Please fix the issues above.")
        print("See README.md for detailed setup instructions.")
    
    return all_passed

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
