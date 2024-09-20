import rclpy
from GazeboEnv import GazeboEnv
import time

def main():
    rclpy.init()

    env = GazeboEnv('simulation.launch.py') 

    try:
        print("Testing reset method...")
        initial_state = env.reset()
        print(f"Initial state after reset: {initial_state}")
        print("Simulating some actions...")
        print("Press Ctrl+C at any time to stop the simulation.")
        while True:
            action = [0.1, 0.1, 0.1]  
            state = env.step(action)
            print(f"State after action: {state}")
            time.sleep(1)  
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down the simulation...")

    finally:
        env.turn_off()
        rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()