Noise-Filter (For UMass Boston's indoor-localisation project)
-----------------
Provided a set of locations which might or might not be the true locations of the object, the program will be able to filter out some noise and return a list of corrected values


Notes:
--------
- This API should only be used to track an object whose motion could be described with a fixed model (In other words, the moving object must belong to a (linear) dynamical system).

Problem needs solving
---------
- In real life, it's hard to find a fixed model to describe an object's motion. That is, most 'natural' motions (a person walking, for instance) are relatively unpredictable, and can't always be correlated to a fixed set of equations!
Other kind of filter(s) would be needed....
- (Should replace the Kalman Filter with something else?) 

Current Achievement:
----------
http://www.cs.umb.edu/~vngu0510/java/kalmanfilter/data/graphs/SampleGraph.pdf
