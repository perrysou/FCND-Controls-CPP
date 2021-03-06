import math;

def fmt(value):
    return "%.3f" % value

period = 20
radius = 1.5
timestep = 0.1
maxtime = period*1
z = -1

with open('HelixFF.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius;
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));
        if t == 0:
            the_file.write("," + fmt(0) + "," + fmt(0) + "," + fmt(0));
        else:
            the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep));
        the_file.write("\n");
        px = x;
        py = y;
        pz = z;
        t += timestep;
        z -= 0.01
