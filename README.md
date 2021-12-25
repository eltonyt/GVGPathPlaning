# GVGPathPlaning
GVG Map for Path Planning

General Idea:
	1. Use OpenCV2 to read the image, and detect Contour of all abstacles(Color in black). For each obstacle, detect vertices it. (Ref: [1])
	2. Scan the whole image, and find the distance between the freespace pixel(Color in white) and all line segment. Distance is calculated using the coordinates of the pixel and vertices. (Ref: [2])
	   If there are distance to 2 line segments(these 2 line segments have to be from different obstacles) that are equal and the distance is the minimum, which means that the pixel is the mid point to the closest 2 obstacles, then this point is a GVG point and will be added to the list.
	   (Note: I used a threshold as 1 in when comparing the distance. If the difference between 2 rounded distance is within 1, then I say they are equal)
	3. For path planing, I used A* algorithm (Ref: [3]). I will search the path for 3 line segments, including start point to GVG map, GVG map to end point, and the path on GVG map. 

Planing Time:
	world1.png is a 272 X 371 image, it takes 1:12 to finish GVG Map Finding & Path Planning.
	world2.png is a 272 X 371 image, it takes 1:02 to finish GVG Map Finding & Path Planning.

Performance:
	1. GVG Creation:
		a. Find contour - my algorithm looks at each pixel at least once. When it looks at each pixel, it looks the surrounded pixels to check whether that's the edge(8 pixels surrounded).
		   Complexity o(8n) -> O(n)
		b. Find vertices - my algorithm looks at each pixels on contours and check whether it's a vertex. Worst senario will be the image is a obstacle and it needs to scan all pixel again.
		   Complexity o(n) -> O(n)
		c. Distance Calculation - my algorithm looks at each point on the freespace and calculate the distance from it to each line segment of obstacles. Worst senario should be the case that we have tons of obstacles.
		   Thus, if we have n pixels for the image, we can at most have n/2 obstacles, and that's the time when each obstacle takes 1 pixel. For each obstacle, there are 4 line segments.
		   Complexity o(1/2 * 4 * n + 4) -> O(n)
		Summary: In total, we will have o(8n + n + 2n) = o(11n) -> O(n) as the complexity
	2. Path Planning:
		Since A* is used for 3 parts of path planing, and A* has O(b^d) as the complexity where d is the shortest path and b is the branching factor. Then the complexity will be o(3*b^d) -> O(b^d)

Path Length:
	1. Start point to GVG map
		The point start point will try to access on GVG map is the point that has shortest distance from start point. 
		I simply used the shortest length between 2 points equation and ignored obstacles to find the point start point should try to access to.
		However, the path from the start point to the point it tris to access to is the shortest since I used A* for path planning.
	2. Path on GVG map
		Path is the shotest path on the GVG map.
	3. GVG map to End Point
		The point start point will try to access on GVG map is the point that has shortest distance from start point. 
		I simply used the shortest length between 2 points equation and ignored obstacles to find the point start point should try to access to.
		However, the path from the start point to the point it tris to access to is the shortest since I used A* for path planning.

References:
	1. Use OpenCV2 to find contours and vertices of contours, https://www.geeksforgeeks.org/find-co-ordinates-of-contours-using-opencv-python/
	2. Find distance between points and line segments, https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	3. A-Star (A*) Search Algorithm, https://towardsdatascience.com/a-star-a-search-algorithm-eb495fb156bb