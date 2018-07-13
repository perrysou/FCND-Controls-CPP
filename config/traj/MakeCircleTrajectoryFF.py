import math;

def fmt(value):
    return "%.3f" % value

period = 4
radius = 1.5
timestep = 0.02
maxtime = period*3

with open('CircleFF.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius;
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + "-1");
        if t == 0:
            the_file.write("," + fmt(0) + "," + fmt(0) + "," + fmt(0));
        else:
            the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt(0));
        the_file.write("\n");
        px = x;
        py = y;
        t += timestep;
