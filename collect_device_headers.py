#!/usr/bin/env python3
import os
import re
from pathlib import Path

def parse_header_file(file_path):
    """Parse a header file to extract function declarations and variables."""
    functions = []
    variables = []
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return functions, variables
    
    # Remove comments and preprocessor directives for cleaner parsing
    content = re.sub(r'//.*', '', content)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    content = re.sub(r'#.*', '', content)
    
    # Function pattern: return_type function_name(parameters);
    function_pattern = r'\b([a-zA-Z_][a-zA-Z0-9_]*\s+[*]*)([a-zA-Z_][a-zA-Z0-9_]*)\s*\([^)]*\)\s*;'
    function_matches = re.findall(function_pattern, content)
    
    for return_type, func_name in function_matches:
        functions.append(f"{return_type.strip()} {func_name}")
    
    # Variable pattern: type variable_name;
    # Look for extern variables and simple declarations
    var_pattern = r'\b(?:extern\s+)?([a-zA-Z_][a-zA-Z0-9_]*\s+[*]*)([a-zA-Z_][a-zA-Z0-9_]*)\s*;'
    var_matches = re.findall(var_pattern, content)
    
    for var_type, var_name in var_matches:
        # Filter out function declarations that might match
        if var_name not in [f.split()[-1] for f in functions]:
            variables.append(f"{var_type.strip()} {var_name}")
    
    return functions, variables

def collect_device_headers():
    """Collect header files from device directories that match their directory name."""
    devices_path = Path("Core/devices")
    
    if not devices_path.exists():
        print(f"Devices directory not found: {devices_path}")
        return
    
    results = {}
    
    # Iterate through each subdirectory in devices
    for device_dir in devices_path.iterdir():
        if device_dir.is_dir():
            device_name = device_dir.name
            header_file = device_dir / f"{device_name}.h"
            
            if header_file.exists():
                print(f"\nProcessing: {header_file}")
                functions, variables = parse_header_file(header_file)
                results[device_name] = {
                    'file': str(header_file),
                    'functions': functions,
                    'variables': variables
                }
    
    # Display results
    print("\n" + "="*80)
    print("DEVICE HEADER ANALYSIS RESULTS")
    print("="*80)
    
    for device_name, data in results.items():
        print(f"\n📁 Device: {device_name.upper()}")
        print(f"📄 File: {data['file']}")
        print("-" * 60)
        
        if data['functions']:
            print("🔧 Functions:")
            for func in data['functions']:
                print(f"  • {func}")
        else:
            print("🔧 Functions: None found")
        
        if data['variables']:
            print("\n📊 Variables:")
            for var in data['variables']:
                print(f"  • {var}")
        else:
            print("\n📊 Variables: None found")
        
        print()

if __name__ == "__main__":
    collect_device_headers()