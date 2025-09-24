import numpy as np
import time

def decision_step(Rover):
    # Storing rover path
    # Storing rover path every 12 steps
    if not hasattr(Rover, 'step_count'):
        Rover.step_count = 0
    Rover.step_count += 1

    if Rover.step_count % 12 == 0:
        x, y = int(Rover.pos[0]), int(Rover.pos[1])
        if (x, y) not in Rover.path_history:
            Rover.path_history.append((x, y))
            
    ## When done flag is true 
    if Rover.done:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = 0
        print("Done Mapping")
        return Rover

    # Move forward every 15 seconds to avoid moving in circles
    current_time = time.time()
    if current_time - Rover.last_forward_time > 15:  
        Rover.steer = 0
        Rover.throttle = 1
        Rover.brake = 0
        Rover.last_forward_time = current_time
        return Rover
    
    # Add current position to memory
    Rover.position_memory.append((round(Rover.pos[0], 3), round(Rover.pos[1], 3)))
    if len(Rover.position_memory) > 15:
        Rover.position_memory.pop(0)

    # Check if rover is stuck (not moving much over last 10 frames)
    if len(Rover.position_memory) == 15:
        unique_positions = set(Rover.position_memory)
        if len(unique_positions) <= 2:  # basically hasn't moved
            print(len(unique_positions), "unique positions in last 10 steps, rover might be stuck")
            Rover.steer = 15        # turn left
            Rover.throttle = -3   # reverse
            Rover.brake = 0 
            return Rover


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and not Rover.picking_up:
        Rover.send_pickup = True
    
    if Rover.rock_angles is not None and len(Rover.rock_angles) > 0:
        # If a rock sample is detected, prioritize approaching it
        rock_angle = np.mean(Rover.rock_angles)
        Rover.steer = np.clip(rock_angle * 180/np.pi, -15, 15)
        if Rover.near_sample:
            Rover.throttle = 0
            Rover.pickup = True
        elif Rover.vel < 1.0:  # Approach slowly
            Rover.throttle = 0.2
        else:
            Rover.throttle = 0
        return Rover

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # If 95% of the map has been covered or, all samples has been collected 
        # Return to home
        if Rover.samples_collected == Rover.samples_to_find or Rover.percentage_mapped >= 98:
            Rover.mode = 'home'
        # If we are near a sample, take it
        if Rover.near_sample:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'stop'
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward and np.mean(Rover.nav_dists) >= 10:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                # --- Wall crawl to the left: steer toward leftmost angle ---
                left_angle = np.percentile(Rover.nav_angles, 85)# leftmost navigable angle (in radians)
                mean_angle = np.mean(Rover.nav_angles)
                # Weighted average: bias toward left wall
                target_angle = 0.5 * left_angle + 0.4 * mean_angle
                if Rover.vel >= 0:
                    # Prevent steering change when rover is moving backwards
                    Rover.steer = np.clip(target_angle * 180/np.pi, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
            else:
                # Set throttle to 0 to coast
                Rover.throttle = 0
                Rover.brake = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if Rover.near_sample and Rover.vel != 0:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                if len(Rover.nav_angles) < Rover.go_forward:
                    # Not enough navigable terrain — STOP
                    Rover.throttle = 0
                    
                    # If the rover is still moving, apply brakes
                    if Rover.vel > 0.2:
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    else:
                        # Fully stopped — now decide how to escape
                        Rover.brake = 0

                        # Option 1: move backwards a bit
                        Rover.throttle = -2   # reverse throttle

                        # Option 2: turn sharply while backing up (like a K-turn)
                        if len(Rover.nav_angles) > 0:
                            Rover.steer = np.clip(np.max(Rover.nav_angles * 180/np.pi), -15, 15) 
                        else:
                            Rover.steer = 15  # default turn if blind
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
            
        elif Rover.mode == 'home':
            print("Returning Home")
            home_x, home_y = Rover.home_pos
            dx = home_x - Rover.pos[0]
            dy = home_y - Rover.pos[1]
            home_distance = np.sqrt(dx**2 + dy**2)
            
            if Rover.backtrack_path == None:
                Rover.backtrack_path = Rover.path_history.copy()[::-1]  # Copy in reverse

            # Pick the next waypoint in the reversed list
            if Rover.backtrack_path:
                target = Rover.backtrack_path[0]
                dx = target[0] - Rover.pos[0]
                dy = target[1] - Rover.pos[1]
                distance = np.sqrt(dx**2 + dy**2)
                angle_to_target = np.arctan2(dy, dx)
                rover_yaw_rad = np.deg2rad(Rover.yaw)
                angle_diff = (angle_to_target - rover_yaw_rad + np.pi) % (2*np.pi) - np.pi

                if distance < 2.0:  # Reached waypoint
                    Rover.backtrack_path.pop(0)
                else:
                    Rover.steer = np.clip(angle_diff * 180/np.pi, -15, 15)
                    if Rover.vel < Rover.max_vel:
                        Rover.throttle = Rover.throttle_set
                    else:
                        Rover.throttle = 0
                    Rover.brake = 0
                if len(Rover.nav_angles) < Rover.stop_forward: # If there is no navigable terrain in front of rover
                    Rover.steer = -np.clip(angle_diff * 180/np.pi, -15, 15) # Negative of required angle when reversing
                    Rover.throttle = -2
                    Rover.brake = 0
                    return Rover
            
            if home_distance < 2.0:  # within 2 meters
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                print("Finished Mapping")
                Rover.mode = 'stop'
                Rover.done = 1

            

    # Just to make the rover do something 
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    
    return Rover