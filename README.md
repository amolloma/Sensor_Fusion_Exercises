# Sensor Fusion Exercises

This repo contains the code for demos, exercises, and exercise solutions.

This repository organizes the code by the lessons that they are used in. Each set of code is located in their respective lessons, except for the primary `basic_loop.py` file that can run each exercise.

Please note that certain instructions for each exercise are only provided within the Udacity classroom.

## Example:
All lesson 1 files are in `/lesson-1-lidar-sensor/`.

This directory contains: `examples`, `exercises/starter`, and `exercises/solution`.

## Environment

Udacity students can make use of the pre-configured workspace environment within the classroom. Alternatively, you can create an environment using the `requirements.txt` file included in this repository, using a command like `pip install -r requirements.txt` if you have pip installed, or creating an Anaconda environment in similar fashion.

### Waymo Open Dataset Reader
The Waymo Open Dataset Reader is a very convenient toolbox that allows you to access sequences from the Waymo Open Dataset without the need of installing all of the heavy-weight dependencies that come along with the official toolbox. The installation instructions can be found in `tools/waymo_reader/README.md`. 

### Waymo Open Dataset Files
This course makes use of three different sequences to illustrate the concepts of object detection and tracking. These are: 
- Sequence 1 : `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`
- Sequence 2 : `training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord`
- Sequence 3 : `training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord`

To download these files, you will have to register with Waymo Open Dataset first: [Open Dataset – Waymo](https://waymo.com/open/terms), if you have not already, making sure to note "Udacity" as your institution.

Once you have done so, please [click here](https://console.cloud.google.com/storage/browser/waymo_open_dataset_v_1_2_0_individual_files) to access the Google Cloud Container that holds all the sequences. Once you have been cleared for access by Waymo (which might take up to 48 hours), you can download the individual sequences. 

The sequences listed above can be found in the folder "training". Please download them and put the `tfrecord`-files into the `dataset` folder within the repository.

# L2 Examples

It's important to note that certain examples in this lesson make use of the outputs of other examples.

For instance, Example C2-3-3 (and Exercise C2-3-2 before it) use the output of Example C2-3-1.

Otherwise, it is often preferred to comment back out previous examples/exercises so you don't have to cycle through all of the related visualizations.

## Use the Escape Key!

To traverse frames, you may need to use the Escape key to progress frames. In limited instances, closing a given frame visualization by clicking on the X of the window may also traverse frames.

## Special Note

In Example C2-4-3, add `True` as an additional argument to `render_bb_over_bev()` for visualization, but remove it when using Example C2-4-4.