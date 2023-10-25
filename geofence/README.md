# Geofence - Using The Winding Number Algorithm

The `geofence.lua` code was used as a security measure in the competition and in the tests carried out. It is based on the Winding Number algorithm. This algorithm was chosen because, despite being computationally heavier than others (Crossing Number algorithm, for example), at the scale we were at (polygons with few sides) it did not make a significant difference. The code also ran on the Pixhawk 6 controller itself and without memory or processing problems. To explain the logic behind the script we will use the [article published by Dan Sunday](geofence-using-the-winding-number-algorithm.md#references).

## Geometric Demonstration

We will give a brief explanation of how the Winding Number algorithm works geometrically. Imagine, then, that you are inside a polygon and need to follow its perimeter with your eyes. Note that if you are inside the polygon, you will make at least one revolution around yourself. Otherwise, you will make zero complete turns. The winding number is exactly this number of turns. If it is 0, you are outside the polygon, otherwise you are inside it. This may remind you of some Calculus class about vector fields in closed curves.

<figure><img src="assets/Winding_Number_Animation_Small.gif" alt=""><figcaption><p><em>Intuitive demonstration of the winding number. The curve winds twice around the person. Imagine that you are inside the polygon, you will not make a full turn.</em></p></figcaption></figure>

Fortunately, we won't need to use calculus to prove this algorithm, just use vector product. If you're curious, see Dan Sunday's article. Let there be a polygon P with n sides and a point Q in the _xy_ plane. Imagine that you are traveling clockwise through a polygon, walking along its sides from one vertex to another. Let us draw a ray parallel to the _x_ axis leaving the point to be analyzed in the positive direction of the _x_ axis. Now, imagine that on your way around the perimeter you have a task: count how many times you cross the ray, adding +1 if you cross the ray from below to above (going up the _y-_axis) and -1 if you cross from above to below

&#x20;(descending on the _y_-axis). The total some is the winding number. In the example below, we only get the absolute value because we disregard the difference in clockwise or counterclockwise turns.&#x20;

<figure><img src="assets/Screenshot from 2023-10-25 02-43-22.png" alt="" width="563"><figcaption><p><em>See how a point inside the polygon has a winding number different from zero. Note that if Q  is inside the polygon 2 times, as if in a fold, the winding number is also 2.</em></p></figcaption></figure>

## Translating into code



How do we do this analysis in Lua code? First, you must define a polygon as a linked list between its vertices. In the code, a table is created for this. The points must be placed **clockwise** on the table. That is, the first point can be any, but the second must be the next one clockwise and so on. Also, put the coordinates in absolute value. Note that the code was created to analyze areas that do not exceed one hemisphere, making it acceptable to work only with absolute values. Once this is done, the script will receive the drone's position from the flight controller and, at each iteration, check whether the drone is within the area or not. If it is outside, some action must be taken, be it landing the drone, disarming it, etc.&#x20;

To perform the verification, we first check each pair of neighboring vertices of the polygon A, B, with A being the vertex before B in a clockwise direction.Then, it is checked whether the latitude of vertex A is smaller or larger than the latitude of the current position. If it is less than or equal, we enter another condition:&#x20;

1. if vertex B has a latitude greater than the latitude of the current position
2. &#x20;if the vector product between A,B with Q (current position) as origin is greater than 0

If these two conditions are satisfied, we add +1 to the winding number. Let's break that down. The (1) condition implies that, to continue the analysis, we have to verify if the ray we drew in the geometric demonstration would cross the side between vertex A and B. That is because,if the condition is met, it means that one of the vertexes is below the Q point and the other one is above Q, so the ray could possibly cross the side if that side is on the right of the point. To know if the side is on the right of the point, we can do the cross product, (2) condition. To illustrate, imagine two vectors, _p1_, from Q to A, and _p2_, from Q to B. If the cross product _p2_ X _p1_, is positive, it means _p1_ is counterclockwise from _p2_. If the cross product is negative, it means _p1_ is clockwise from _p2_.&#x20;

<figure><img src="assets/Screenshot from 2023-10-25 13-53-38.png" alt="" width="563"><figcaption><p><em>Remember that the ray is in the direction Q -> positive x-axis</em></p></figcaption></figure>

In case **II**, we add +1 in the winding number, because the side AB is on the right of the point. Note that it is as if we were crossing the ray from below to above, just like in the geometric demonstration. In case **I**, see that the ray would not cross the side AB, so we do not add neither subtract from the winding number.

An analogous analysis is made for the case in which the latitude of vertex A is greater than that of Q. In this case, the two new conditions are:

1. if vertex B has a latitude smaller than the latitude of the current position
2. &#x20;if the vector product between A,B with Q (current position) as origin is smaller than 0

<figure><img src="assets/Screenshot from 2023-10-25 13-53-47.png" alt="" width="557"><figcaption><p><em>Once more, imagine the ray going from Q to the positive x-axis</em></p></figcaption></figure>

In case **IV**, we subtract -1 in the winding number, because the side AB is on the right of the point. Note that it is as if we were crossing the ray from above to below, just like in the geometric demonstration. In case **III**, see that the ray would not cross the side AB, so we do not add neither subtract from the winding number.

Thus, we finish the explanation of the winding number algorithm that was applied in the Lua script.

## How to use the script

To use the script, create tables with the polygons of the _geofences_ and apply the linked list function to each one of them, passing the table and the number of vertices as arguments. Then, in the `geoVerify`create conditions for each calculated winding number. Remember to list the vertices in the clockwise order. To avoid taking emergency measures (such as turning off the motors) due to GPS errors or inaccuracy, we recommend to implement a counter for each time that the drone has been outside the safe zone. Change the update time of the `geoVerify` function as necessary.&#x20;

## References:

[https://pd.daffodilvarsity.edu.bd/course/material/book-430/pdf\_content](https://pd.daffodilvarsity.edu.bd/course/material/book-430/pdf\_content)&#x20;

[http://profs.ic.uff.br/\~anselmo/cursos/CGI/slidesNovos/Inclusion%20of%20a%20Point%20in%20a%20Polygon.pdf](http://profs.ic.uff.br/\~anselmo/cursos/CGI/slidesNovos/Inclusion%20of%20a%20Point%20in%20a%20Polygon.pdf)

