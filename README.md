# Detection of Object and Tool Affordances through RGB-D Sensing and Robot Exploration
This is a catkin workspace containing the simulation environment and Bayesian network models for affordance learning.

## Prerequisites
You will need the following software installed on your machine:
- ROS Kinetic
- Gazebo 8
- RViz
- MoveIt!
- RStudio
- Python packages: numpy, matplotlib, sklearn, pandas, OpenCV
- R packages: bnlearn, Rgraphviz, gRain

## Getting Started
First clone the catkin workspace

`git clone https://github.com/yiklungpang/master_project.git`

Once the repository is downloaded, run
```
cd master_project
catkin_make
source devel/setup.bash
```
## Data Collection
The data collection process contains 2 steps:
- Record visuals (RGB and point clouds)
- Perform affordance experiment and record results

You can start the data collection process by running:

`roslaunch affordance_experiment data_collection.launch`

RGB images and PCD files of point clouds will be saved in:

`master_project/src/affordance_experiment/visuals/data`

Experiment results will be saved as JSON in:

`master_project/src/affordance_experiment/bn/data`

## Generating Visual Features
To generate 2D features go to:

`master_project/src/affordance_experiment/visuals`

and run:

`python rgb_features.py`

You can display the feature extraction process by adding the debug flag `-d`:

`python rgb_features.py -d`

2D features will be saved as a csv file in:

`master_project/src/affordance_experiment/bn/features/rgb_features.csv`

To generate 3D features go to:

`master_project/src/affordance_experiment/visuals/build`

and run:

`./pc_features`

You can display the extracted point clouds by adding the debug flag `-d`:

`./pc_features -d`

3D features will be saved as a csv file in:

`master_project/src/affordance_experiment/bn/features/pc_features.csv`

## Generating Scatter Plots
To generate the scatter plots, go to:

`master_project/src/affordance_experiment/bn`

and run:

`python scatter_plot.py`

## Preprocess Features and Experiment Data
Before training and testing the BNs, the features and experiment data generated must be preprocessed and outputted to a csv file.
To run the preprocessing, go to:

`master_project/src/affordance_experiment/bn`

and run:

`python preprocess_final.py`

Preprocessed data will be saved to:

`master_project/src/affordance_experiment/bn/processed_data`

## Training and testing the Bayesian Networks
Each network has a separate R script as they all have different structures.

2D Fully connected: `fully_connected_2d.r`

2D PCA: `pca_2d.r`

2D BDe: `bde_2d.r`

3D Fully connected: `fully_connected.r`

3D PCA: `pca.r`

3D BDe: `bde.r`

These should be run in RStudio as the results are stored as variables.

Seen tool randomised prediction percentage: `random_rp`

Seen tool accuracy: `random_accuracy`

Seen tool gambling score: `random_gambling_score`

Seen tool distance measure: `random_distance`

Unseen tool randomised prediction percentage: `loo_rp`

Unseen tool accuracy: `loo_accuracy`

Unseen tool gambling score: `loo_gambling_score`

Unseen tool distance measure: `loo_distance`
