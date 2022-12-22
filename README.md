# supertoroid_paper
new version of the supertoroid repo that is used for the paper

# Explanation
The main three nodes are:

sampling_test_pcd_st
pcd_viewer
st_fitting_test_pcd

The way the fitting works is the following:

- load the point cloud
- calls fit()    <-located in st_fitting.cpp
        set PreAlign     <- this one only defines the index to do the rotation of the x,y,z axes that I explained to you
        fit_param               <- this one calls a series of functions:
                preAlign        <- applies PCA to point cloud, finds centroid and principal directions, creates transform matrix, transforms point                              cloud to local supertoroid coordinates (prealigned_cloud).
                load initial conditions (from yaml file)
                performs LM optimization (function is defined in operator, which uses st_function_b1 and st_function_b2, which is the                   calculation of the betas)
                calculates error

# Usage
run the following launch files:

STsampling.launch
STfitting.launch

To change parameters: STparameters.yaml