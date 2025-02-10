import numpy as np

def upd_pos_vel(Ncar, pos, vel, acc, headway, dv, posnew, velnew, params):

    des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t = params[:7]
    
    # Loop over all cars
    for i in range(Ncar):

        # Calculate desired bumper-to-bumper distance (s*)
        s_star = min_gap + max(0, vel[i] * time_gap + (vel[i] * dv[i]) / (2 * (acc_max * comf_decel) ** 0.5))

        # Calculate acceleration using IDM
        acc[i] = acc_max * (1 - (vel[i] * des_speed_inv)**acc_exp - (s_star / headway[i])**2)

        # Update velocity and position using Newtonian dynamics
        velnew[i] = vel[i] + acc[i] * del_t
        posnew[i] = pos[i] + vel[i] * del_t + 0.5 * acc[i] * del_t**2

        # Ensure velocity does not go negative
        if velnew[i] < 0:

            # Calculate time to stop
            t_stop = -vel[i] / acc[i]

            # Update position and velocity to stop at t_stop
            posnew[i] = pos[i] + 0.5 * vel[i] * t_stop
            velnew[i] = 0

    return posnew, velnew, acc

def detect_loop(Ncar,pos,vel,acc,posnew,velnew, detect_time,detect_vel, det_point, time_pass):

    for i in range(Ncar):

        # Check if the car has passed the detection point
        if pos[i] < det_point <= posnew[i]:

            # Linear interpolation to find the exact time and speed
            delta_t = (det_point - pos[i]) / vel[i]
            detect_time[i] = time_pass + delta_t
            detect_vel[i] = vel[i] + acc[i] * delta_t

    return detect_time, detect_vel

def flow_global(N,velnew,L):

    # Calculate global density (cars per km)
    dens = N / (L / 1000)
    
    # Calculate global flow (cars per hour)
    flow = np.mean(velnew) * dens * 3600 / 1000

    return dens, flow

def update_cars(Ncar, pos, vel, posnew, velnew, headway, dv, params):

    # Extract the key parameters from params
    length, L, min_gap  = params[7], params[8], params[4]

    # Update positions and velocities
    for i in range(Ncar):
        pos[i] = posnew[i] % L
        vel[i] = velnew[i]

    # Update headway and velocity difference
    for i in range(Ncar):
        next_car = (i + 1) % Ncar

        # Calculate headway, accounting for the length of the cars
        headway[i] = (pos[next_car] - pos[i] - length) % L

        # Ensure the minimum gap is maintained
        if headway[i] < min_gap:
            headway[i] = min_gap

            # Adjust the position of the current car to maintain the minimum gap
            pos[i] = (pos[next_car] - min_gap - length) % L
            
        # Update velocity difference
        dv[i] = vel[next_car] - vel[i]

    return pos, vel, headway, dv

def Step(N,pos,vel,headway,dv,params,time_pass,time_measure,L):

        posnew=np.zeros(N)
        velnew=np.zeros(N)
        acc= np.zeros(N)
        den = 0
        flo = 0

        posnew, velnew, acc = upd_pos_vel(N, pos, vel, acc, headway, dv, posnew, velnew, params)

        if (time_pass > time_measure):

            #detect_time, detect_vel = detect_loop(N, pos, vel, acc, posnew, velnew, detect_time, detect_vel, det_point, time_pass)
            den, flo = flow_global(N, velnew, L)

        pos, vel, headway, dv = update_cars(N, pos, vel, posnew, velnew, headway, dv, params)

        return pos, vel, headway, dv, den, flo #, detect_time, detect_vel

def init_params():

    # TODO add det_point (location of a detection point - not needed initially)
    Nmax, L, steps, steps_measure, splim = 100, 1000, 1000, 20, 40 # max_cars, len_road, steps_num, steps_bef_measure, speed_limit 

    params = [1./splim, 2, 2, 2, 2, 3, 2, 3, L]  # des_speed_inv, acc_exp, time_gap, comf_decel, min_gap, acc_max, del_t, length, L

    return Nmax, L, steps, steps_measure, splim, params

def init_simulation(N,L):

  # Initial conditions
    vel = np.zeros(N)  # Initial velocity (all cars stationary)
    pos = np.linspace(0, L, N, endpoint=False)  # Initial positions (equally spaced)
    dv = np.zeros(N)  # Speed differential
    headway = np.full(N, (L / N) - init_params()[5][7])  # Headway (distance to preceding car)

    print(N, 'Cars initialised')
    return vel, pos, dv, headway

def analyse_global(track_flow, track_dens):

    # Calculate overall global flow and density
    glob_flow = np.mean(track_flow)
    glob_dens = np.mean(track_dens)
    
    return glob_flow, glob_dens

def analyse_local(track_det_time, track_det_flow):
    loc_flow, loc_dens = 0,0
    return loc_flow, loc_dens

def Simulate_IDM(N,params,steps,time_measure):
  
  L = params[8]
  del_t = params[6]
  track_det_time=[]
  track_det_vel=[]
  track_flow=[]
  track_dens=[]

  vel, pos, dv, headway = init_simulation(N,L)

  for i in range(steps):

    time_pass = i * del_t

    if (time_pass > time_measure):

        pos, vel, headway, dv, den, flo = Step(N,pos,vel,headway,dv,params,time_pass,time_measure,L)

        track_det_time.append(del_t)
        track_det_vel.append(dv)
        track_flow.append(flo)
        track_dens.append(den)

    else:
        pos, vel, headway, dv, den, flo = Step(N,pos,vel,headway,dv,params,time_pass,time_measure,L)

  glob_flow, glob_dens = analyse_global(track_flow, track_dens)
  #loc_flow, loc_dens = analyse_local(track_det_time, track_det_flow)
  print(f'Simulation for car total {N} completed\n')

  return glob_flow, glob_dens #, loc_flow, loc_dens