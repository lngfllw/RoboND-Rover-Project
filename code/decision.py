import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        dist_th = 50 #this is a threshold to distinguish close from far
        dist = Rover.nav_dists
        angles = Rover.nav_angles
        print('mean dist ', np.mean(dist))
        print(Rover.mode)
        
        
        #check if rover is stuck
        if Rover.throttle > 0 and Rover.vel < .01 and Rover.picking_up ==0:
            Rover.stuck_count += 1
            print('stuck count ', Rover.stuck_count)
        if Rover.stuck_count > 35:
            #flag rover as stuck
            Rover.mode = 'stuck'
            # Set mode to "stop" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_set
            Rover.steer = 0
         
        #check if rover near sample
        if Rover.see_rock == 1:
            Rover.mode = 'rock'
            
        print('mode: ', Rover.mode)
        
        
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:# and np.mean(dist) >= 38:
                #  np.mean(dist) >= dist_th: #if there is a lot of nav ahead 
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                
                
                # Set steering to average angle clipped to the range +/- 15
                #make a wall crawler
                #focus on ground that is close to avoid crashing into boulders
                
                
                #prepare to populate close dist ang mats
                dist_cl_0 = np.zeros_like(dist)
                ang_cl_0 = dist_cl_0

                #record dist, ang that are close
                for i in range(dist.shape[0]):
                    if dist[i] < dist_th:
                        dist_cl_0[i] = dist[i]
                        ang_cl_0[i] = angles[i]
                       
                #get rid of zero parts, we have mats of dist, ang that are close
                #indices of nonzero parts
                dist_nz = np.flatnonzero(dist_cl_0)
              #  print(len(dist_nz))
                ang_nz = np.flatnonzero(ang_cl_0)
                #mats will be populated 
                dist_cl = np.zeros(len(dist_nz))
                ang_cl = np.zeros(len(ang_nz))
                #transfer nonzero from whole mats into nonzero only mats
                for i in range(len(dist_cl)):
                    dist_cl[i]=dist[dist_nz[i]]
                    ang_cl[i]=angles[ang_nz[i]]
                    
                #ang_cl = np.flatnonzero(ang_cl)
                ang_cl_mn = np.mean(ang_cl)
                ang_cl_min = np.min(ang_cl)
                ang_cl_steer = (3*ang_cl_mn + ang_cl_min)/4
                
                ang_mn = np.mean(angles)
                ang_min = np.min(angles)
                ang_steer = (3*ang_mn + ang_min)/4
                
                if ang_cl_min > -.1:
                    Rover.steer = 15
                elif len(Rover.nav_angles) > 5000:
                    Rover.steer= -3
                elif len(dist_nz) < 800: #if obstacle near rover
                    Rover.steer = np.clip(np.mean(ang_cl_steer * 180/np.pi), -15, 15)
                else: #if no obstacle in front
                    Rover.steer = np.clip(np.mean(ang_steer * 180/np.pi), -15, 15)
                
                
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:# and np.mean(dist) < 38:
                #np.mean(dist) < dist_th: #if there is a lot of nav ahead 
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
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
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    
        elif Rover.mode == 'stuck':
        #turn for a set amount of iterations then try forward again
            print(' in stuck mode loop')
#        if len(Rover.nav_angles) < 3 * Rover.go_forward:
            #count down for rover to turn around
            Rover.stuck_count -= 1
            
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = 15 # Could be more clever here about which way to turn
            Rover.mode = 'stuck'
        # If we're stopped but see sufficient navigable terrain in front then go!
            if Rover.stuck_count < 10: #try again
                #len(Rover.nav_angles) >= 3 * Rover.go_forward:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.stuck_count = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.mode = 'forward'
                
                
        elif Rover.mode =='rock':
            rock_d = Rover.rock_dist_ang[0]
            ang_d = Rover.rock_dist_ang[1]
            print('rock dist = ', rock_d)
            if rock_d > 8:
                Rover.steer = np.clip(ang_d*180/np.pi, -15, 15)
                Rover.throttle = Rover.throttle_set*0.5
                Rover.brake = 0
                print('steer, throttle ', Rover.steer, Rover.throttle)
            elif rock_d <= 8: 
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.send_pickup = True
                print('pickup sent')
                Rover.picking_up = 0
            
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    
    else:
        Rover.throttle = 0
        Rover.steer = 15
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
#    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
#        Rover.send_pickup = True
    
    return Rover


