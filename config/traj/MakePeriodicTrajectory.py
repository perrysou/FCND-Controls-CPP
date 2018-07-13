import math;

def fmt(value):
    return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period)*3
timemult = [1, 1, 1]
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]

with open('FigureEight_alt.txt', 'w') as the_file:
    t=0;
    px = 0;
    py = 0;
    pz = 0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0];
        y = math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1];
        z = math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2];
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
		######## BEGIN STUDENT CODE
        if t >= 2 * timestep:
            vx = (x-px)/timestep;
            vy = (y-py)/timestep;
            vz = (z-pz)/timestep;
            ax = (vx-pvx)/timestep;
            ay = (vy-pvy)/timestep;
            az = (vz-pvz)/timestep;
            the_file.write(
            "," + fmt(vx) +
            "," + fmt(vy) +
            "," + fmt(vz) +
            "," + fmt(ax) +
            "," + fmt(ay) +
            "," + fmt(az)
            );
        elif t == timestep:
            vx = (x-px)/timestep;
            vy = (y-py)/timestep;
            vz = (z-pz)/timestep;
            ax = 0
            ay = 0
            az = 0
            the_file.write(
            "," + fmt(vx) +
            "," + fmt(vy) +
            "," + fmt(vz) +
            "," + fmt(ax) +
            "," + fmt(ay) +
            "," + fmt(az)
            );
        elif t == 0:
            vx = 0
            vy = 0
            vz = 0
            the_file.write(
            "," + fmt(0) +
            "," + fmt(0) +
            "," + fmt(0) +
            "," + fmt(0) +
            "," + fmt(0) +
            "," + fmt(0)
            );
        px = x;
        py = y;
        pz = z;
        pvx = vx
        pvy = vy
        pvz = vz
		######## END STUDENT CODE
		######## EXAMPLE SOLUTION
        #the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep));
		#px = x;
        #py = y;
        #pz = z;
		######## END EXAMPLE SOLUTION

        the_file.write("\n");

        t += timestep;
