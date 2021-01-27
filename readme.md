This program determines if an assembly of planar rigid bodies, in frictional contact with each other, can remain standing in gravity or if the assembly must collapse.

The program takes as input:
a description of the static mass properties of each of the N bodies: the (x,y) location of the center of mass and the total mass in kg.
a description of the contacts. Each contact consists of a list of the two bodies involved in the contact (0 means stationary ground); the (x,y) location of the contact; the contact normal direction into the first body involved in the contact; and the friction coefficient mu at the contact.
The output is binary: it is either possible or impossible for the assembly to remain standing. The solution method is linear programming that finds the contact vector k. This represents one set of contact forces that would keep the assembly standing.