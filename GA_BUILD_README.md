# GA ArduPilot Project
![Alt text](https://www.generalaeronautics.com/wp-content/uploads/2021/04/GA-Logo-White-300x90.png)

This is the GA's build/repo of ArduPilot, the most advanced, full-featured and reliable open source autopilot software available.


## Pre-requisites before building the code using Pipelines
Under the ArduCopter folder:

- Update the ReleaseNotes_GA
            with latest changes at the top and the oldest at the end
- Define THISFIRMWARE version in version.h file as per current revision number  
            as "GA ArduCopter V{Arducopter_version} v{GA_version}", i.e. like  
            "GA ArduCopter V4.1.1 v1" or "GA ArduCopter V4.1.1 v1beta3"
  
  
## Pipeline Notes: 
- The Pipeline currently builds for the boards: CubeOrange, CubeBlack & Pixhawk4
- After building, the files and their info can be found as a zip file in the Downloads section of the Bitbucket repository
- The same above, but as a folder gets deployed to AWS S3
  
  
## For branches: Using the Pipeline to only build / build & deploy
The Pipeline can be triggered manually to only build or to build & deploy the GA-ArduPilot code. But, this can only be done for branches (and not for tags). 

To do this, go to the Pipelines section in the Repository and select "Run pipeline"

- Select the required branch  
- In Pipeline, select  
    * custom: BuildAPcopter                     - to just build the code for the above boards  
    * custom: BuildAndDeployAPcopterToAWSS3     - to build the code for the above boards and deploy on AWS S3  
- The .bin & .apj files for the boards can be found in the Downloads section of the Bitbucket repository

## Using tags to build and deploy the code to AWS S3 automatically
To build and deploy the GA-ArduPilot code automatically, tag the specific commit as ga_copter{any_text}, i.e. the tag should start with the text "ga_copter" and this text can be followed by anything. On pushing such tags to this repo, the Pipeline gets triggered automatically.

The pipeline executes and on successful completion keeps a zipped folder containing the builds for above mentioned boards in the Downloads section of the Bitbucket repository. The same is also deployed as a folder in the AWS S3 bucket. 