# PointCloud_Transformation
Contains codes that perform transformations between two point clouds(A&B) 

Implemets Kabsch's algorithm for calculating Rotation and Translation of one set of points to another using Singluar Value Decomposition. 

The transformation matrix obtained is then used on the rest of the points(belonging to A) so that the cloud sets can be superimposed. 

Both the input matrices need atleast 3 independent points. 

Adapted from: http://nghiaho.com/?page_id=671
