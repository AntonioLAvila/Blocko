Big path/trajectory generator library based on 254's 2014 auto code and pathfinder. Has been tested on
Doc 14, supports reverse path traversal, but you have to create the waypoints in a different way. For example, to go forward on a path, you would pass in a list of waypoints in the order and direction of traversal ex: new Waypoint(0.0, 0.0, Math.PI), new Waypoint(-10.0, 0.0, Math.PI)
To go in reverse on that same path, you would write 
    new Waypoint(-10.0, 0.0, 0.0), new Waypoint(0.0, 0.0, 0.0)
then you have to pass in a boolean that will reverse paths for you.

Also, a weird quirk that I (Maverick) might change is that when you are generating the paths, you have to 
instantiate the heading in radians, while following the path requires the measured heading to be in degrees, sorry, but blame engineers 
for measuring angle natively in degrees.

------------------------------

Waypoint coordinates are in whatever values you pass to the calculator. If you're measuring in ft each unit is 1ft last coordinate is angle measure, have fun if I didn't explain this to you my senior year, sorry, just figure it out lol ;) But really I think if you read it you'll understand it, good luck.
-not Maverick