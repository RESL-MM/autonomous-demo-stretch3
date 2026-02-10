#!/usr/bin/env python3
"""
Stress test script for dial twisting demo.
Runs the twist_dial_demo.py multiple times and logs results.
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime

def run_dial_twisting_demo(demo_args, run_number, total_runs):
    """
    Run the dial twisting demo once.
    
    Args:
        demo_args: List of arguments to pass to twist_dial_demo.py
        run_number: Current run number (1-indexed)
        total_runs: Total number of runs
    
    Returns:
        tuple: (success: bool, duration: float, error_message: str or None)
    """
    print(f"\n{'='*60}")
    print(f"Starting Run {run_number}/{total_runs}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}\n")
    
    # Get the directory where this script is located
    script_dir = Path(__file__).parent
    demo_script = script_dir / "twist_dial_demo.py"
    
    # Build the command
    cmd = [sys.executable, str(demo_script)] + demo_args
    
    start_time = time.time()
    try:
        # Run the demo
        result = subprocess.run(
            cmd,
            capture_output=False,  # Show output in real-time
            text=True,
            check=True
        )
        duration = time.time() - start_time
        print(f"\n✓ Run {run_number} SUCCEEDED (Duration: {duration:.1f}s)")
        return True, duration, None
        
    except subprocess.CalledProcessError as e:
        duration = time.time() - start_time
        error_msg = f"Exit code: {e.returncode}"
        print(f"\n✗ Run {run_number} FAILED (Duration: {duration:.1f}s, {error_msg})")
        return False, duration, error_msg
        
    except KeyboardInterrupt:
        duration = time.time() - start_time
        print(f"\n⚠ Run {run_number} INTERRUPTED by user (Duration: {duration:.1f}s)")
        raise
        
    except Exception as e:
        duration = time.time() - start_time
        error_msg = str(e)
        print(f"\n✗ Run {run_number} FAILED with exception (Duration: {duration:.1f}s)")
        print(f"Error: {error_msg}")
        return False, duration, error_msg


def main():
    parser = argparse.ArgumentParser(
        prog='Dial Twisting Stress Test',
        description='Run the dial twisting demo multiple times to test reliability.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run demo 5 times with default settings
  python dial_stress_test.py 5
  
  # Run demo 10 times with custom exposure
  python dial_stress_test.py 10 --exposure medium
  
  # Run demo 3 times with high exposure and custom delay
  python dial_stress_test.py 3 --exposure high --delay 5.0
        """
    )
    
    parser.add_argument(
        'n',
        type=int,
        help='Number of times to run the dial twisting demo'
    )
    
    parser.add_argument(
        '--delay',
        type=float,
        default=2.0,
        help='Delay in seconds between runs (default: 2.0)'
    )
    
    # Forward arguments to twist_dial_demo.py
    parser.add_argument(
        '-e', '--exposure',
        type=str,
        default='low',
        help='Set D405 exposure (forwarded to demo)'
    )
    
    args = parser.parse_args()
    
    # Validate n
    if args.n < 1:
        parser.error("Number of runs must be at least 1")
    
    # Build argument list to forward to twist_dial_demo.py
    demo_args = ['--exposure', args.exposure]
    
    # Print test configuration
    print("\n" + "="*60)
    print("DIAL TWISTING STRESS TEST")
    print("="*60)
    print(f"Total runs: {args.n}")
    print(f"Delay between runs: {args.delay}s")
    print(f"Demo arguments: {' '.join(demo_args) if demo_args else '(none)'}")
    print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    
    # Track results
    results = []
    successful_runs = 0
    failed_runs = 0
    
    try:
        for i in range(1, args.n + 1):
            success, duration, error = run_dial_twisting_demo(demo_args, i, args.n)
            results.append({
                'run': i,
                'success': success,
                'duration': duration,
                'error': error
            })
            
            if success:
                successful_runs += 1
            else:
                failed_runs += 1
            
            # Delay between runs (but not after the last one)
            if i < args.n and args.delay > 0:
                print(f"\nWaiting {args.delay}s before next run...")
                time.sleep(args.delay)
    
    except KeyboardInterrupt:
        print("\n\n⚠ Stress test interrupted by user!")
    
    # Print summary
    print("\n" + "="*60)
    print("STRESS TEST SUMMARY")
    print("="*60)
    print(f"Total runs attempted: {len(results)}/{args.n}")
    print(f"Successful: {successful_runs}")
    print(f"Failed: {failed_runs}")
    
    if results:
        success_rate = (successful_runs / len(results)) * 100
        print(f"Success rate: {success_rate:.1f}%")
        
        durations = [r['duration'] for r in results]
        avg_duration = sum(durations) / len(durations)
        print(f"Average duration: {avg_duration:.1f}s")
        print(f"Min duration: {min(durations):.1f}s")
        print(f"Max duration: {max(durations):.1f}s")
    
    print(f"\nEnd time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    
    # Detailed results
    if failed_runs > 0:
        print("\nFailed runs:")
        for r in results:
            if not r['success']:
                print(f"  Run {r['run']}: {r['error']}")
    
    # Exit with error code if any runs failed
    sys.exit(0 if failed_runs == 0 else 1)


if __name__ == '__main__':
    main()
