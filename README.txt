README.txt

This Readme file was created for a basic overview of how to use the gp_cochlea_estimate.m file to estimate the pose of a
guinea pig cochlea.  It outlines the folder structure and files necessary to run the code as well as a snippet of example
code to test the program.


FOLDER STRUCTURE:
	The folder structure should contain the following in addition to this Readme.txt file:


	- JRMPC
		- All code from the JRMPC library folder contents availble at [https://team.inria.fr/perception/research/jrmpc/] should be placed here
		(ex.)
		- jrmpc.m
		- utils
			- mat2off.m
			- orient_normals.m
			- read_motion_fn.m
			- scan_to_matrix.m
			- setsandparameters.mat
	- gp_cochlea_estimate.m
	- sample_run.m
	- DATA
		- input_data_set.dat ( 10x3 array of values in order in .dat format, see below for example )
		- combined_coords_L_CO.mat
		- combined_coords_R_CO.mat
	- FUNCTIONS
		- convertCoord.m
		

TO RUN:
To run the code, copy the 10x3 array of feature points you will use into input_data_set.dat or create a new .dat file (and change the input file name in sample_run.m) with a 10x3 array of the input GP points in the following order (See example below):

FPI
FPS
LSC
MA
OI
OS
RWFF
RWCF
ST
ZA

Then run the code in Matlab seen below in sample_run.m.


SAMPLE GP FILE:

	0.6768	    1.9356	    0.8123
	0.5107	    2.6278	    0.8198
	1.0900	    4.5171	    0.5487
	6.8294	    7.0181	    4.1665
	6.4841	    4.5931	   -0.2007
	3.3553	    6.8527	    0.6895
	2.5429	    1.9787	    0.3105
	1.2282	    2.1177	    0.4490
	1.4695	    2.6039	    0.9428
	13.897	    9.6757      8.6077
