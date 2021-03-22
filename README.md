
This Readme file was created for a basic overview of how to use the gp_cochlea_estimate.m file to estimate the pose of a
guinea pig cochlea.  It outlines the folder structure and files necessary to run the code as well as a snippet of example
code to test the program.

This code is based off of a paper to be published in Otology and Neurogology.  Link will be provided soon.

This method is based off of the JRMPC algorithm and paper found [here].(https://team.inria.fr/perception/research/jrmpc/)

You can also check out a basic Qt gui created that can be used to record point touching and will output the required file in another repo I made [here](https://github.com/dusevitch/GP_Feature_Selector_GUI). More details are included specifying how/where to select points. NOTE: It uses an NDI Polaris motion tracking device. You'll need to include whichever motion tracking device you're using. 
		

TO RUN:
To run the code, load the the 6x3 array of feature points file you will use (see sample_data_file.dat) into the gp_cochlea_estimate.  The 6 features needed to be touched and recorded by a motion tracking system are (please see the attached paper for reference):

CL, MN, OS, RWA, RWP, and LLI

Then run code as seen in sample_run.m.


SAMPLE_DATA_FILE.DAT:

	-151.719	62.664	-89.2556
	-159.526	66.6298	-86.6652
	-155.467	68.299	-85.1436
	-153.705	65.6777	-86.3773
	-152.624	65.4604	-85.9927
	-153.585	66.0412	-86.3372

