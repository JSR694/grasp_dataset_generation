#!/usr/bin/env python
###############################################################################
#
# Generates grasps for a Baxter gripper and a set of models. Stores them in a database.
#
# Grasp planning with Graspit, by way of graspit_interface and graspit_commander.
#   NOTE graspit_interface must be launched externally.
# Dataset is an HDF5 file, abstracted away by the GraspDataset class.
#
# NOTE: Dataset must be created from scratch.  Currently does not support 
#   editing existing datasets.
#
# Author: Jack Rasiel
#
# TODO add proper ROS-style header (with license etc.)
#
###############################################################################

import rospy
from os import listdir, remove
from os.path import splitext
from graspit_commander.graspit_commander import GraspitCommander as gc
from graspit_commander.graspit_exceptions import GraspitTimeoutException
from graspit_interface.msg import Grasp, Planner, SearchEnergy, PregraspParams
from grasp_dataset import GraspDataset

# Initializes a GraspDataset.  Adds model names to the dataset; 
#   generates grasps for those models, and adds those grasps to the dataset.
class InitialGraspDsetBuilder(object):
    def __init__(self, dset_filepath, dset_config_filepath, 
                 hand_name, models_path = None):
        #rospy.init_node("initial_dataset_generator")

        self.hand_name = hand_name
        self.models_path = models_path
        self.model_names = []
        self.models_subdir = None

        # Load dataset from specified dir.
        self.dset = GraspDataset(dset_filepath, dset_config_filepath)
        rospy.on_shutdown(self.dset.close_dataset)
        rospy.loginfo("Successfully initialized dataset.")
        rospy.loginfo("Dataset location:\t%s" % dset_filepath)
        rospy.loginfo("Dataset config location:\t%s" % dset_config_filepath)
        if self.dset.get_current_model_name() != "":
            rospy.loginfo("Current model name:\t%s" % 
                          self.dset.get_current_model_name())

        # Load model names+dirs and add names to dataset:
        if models_path is not None:
            # Graspit loads models relative to $GRASPIT/models/objects:
            self.models_subdir = models_path.split("models/objects/")[1]
            self.dset.set_metadata("models_subdir",self.models_subdir)

            # Load model names from specified dir.
            try:
                # Get all files, remove exts, and remove duplicates:
                files = listdir(models_path)
                files = map(lambda f: splitext(f)[0], files)
                files.sort()
                self.model_names = [files[i] for i in range(0, len(files),2)]
                rospy.loginfo("Successfully loaded %d models." % len(self.model_names))
            except OSError:
                rospy.logerr("Problem loading models from dir %s" % models_path)
                return

            # Add model names to dset:
            self.dset.add_models(self.model_names)
            rospy.loginfo("Added models to dataset.")
        else:
            # If no new model names added, must have stored models_subdir
            #  to locate previously added models.
            rospy.loginfo("models_path is None.  \
                          No new model names added to dataset.")
            if self.dset.get_metadata("models_subdir") is None:
                rospy.logfatal("No model subdirectory found in dataset.\
                             Unable to load models from names in dataset.")
            else:
                self.models_subdir = self.dset.get_metadata("models_subdir")

        # Set grasp planner parameters.
        #TODO planner preferences as params?
        self.planner_type = Planner(Planner.SIM_ANN)
        self.planner_max_steps = 35000 #40000#100000

        rospy.loginfo("Finished setting up.")

    def generateGraspNextModel(self):
        return self.generateGraspOneModel(
            self.dset.get_current_model_name())

    def generateGraspOneModel(self, model):
        rospy.wait_for_service("clearWorld")
        try: 
            rospy.loginfo("Generating grasps for %s" % model)
            ret = self.generateGrasp_helper(model)
            grasps, energies = ret.grasps, ret.energies

            rospy.loginfo("Storing grasps in dataset.")
            self.dset.add_initial_grasps(grasps, energies, 
                                         is_new_model=True)
        except GraspitTimeoutException:
            rospy.logerr("Graspit commander timed out.  No grasps planned.")
            rospy.logerr("Please ensure that graspit_interface is running.")

    # TODO this is broken bc Graspit will always segfault:
    def generateGraspAllModels(self):
        rospy.wait_for_service("clearWorld")
        try: 
            for model in self.model_names:
                print self.model_names
                ret = self.generateGrasp_helper(model)
                if ret is None: return # if grasp planning fails
                grasps, energies = ret.grasps, ret.energies

                rospy.loginfo("Storing grasps in dataset.")
                self.dset.add_initial_grasps(grasps, energies, 
                                             is_new_model=True)
        except GraspitTimeoutException:
            rospy.logerr("Graspit commander timed out.  No grasps planned.")
            rospy.logerr("Please ensure that graspit_interface is running.")

    # Helper for generateGrasp. Plan grasps for one model.  
    #  Returns a response object w/ grasps, energies, and search energies.
    def generateGrasp_helper(self, model_name):
        if self.models_subdir is not None:
            model_name = self.models_subdir + "/" + model_name
        rospy.loginfo("Generating grasps for model %s" % model_name)
        try:
            rospy.loginfo("Setting up world.")
            gc.clearWorld()
            gc.importRobot(self.hand_name)
            gc.importGraspableBody(model_name)
            rospy.loginfo("Running planner.")
            #TODO testing planner params:
            #response = gc.planGrasps(planner=self.planner_type, 
            #                         max_steps=self.planner_max_steps)
            response = gc.planGrasps(planner=Planner(Planner.MULTI_THREADED), 
                                     max_time=60)
        except rospy.ServiceException:
            print "Problem with GraspitCommander service call.  Killing node."
            self.dset.close_dataset()
            return None
            #rospy.logfatal("Service call failed!  Graspit probably segfaulted.  Killing node.")
            #rospy.signal_shutdown("Graspit service call failed.")
        else:
            return response

# Loads an existing GraspDataset populated with model names and grasps.
#  Generates table grasps for those models+grasps, and adds them to the dataset.
#
#   hand_pregrasp_params is a PregraspParams msg.
class TableGraspDsetBuilder(object):
    def __init__(self, dset_filepath, dset_config_filepath, 
                 hand_name, hand_pregrasp_params, models_path = None,
                 num_model_poses = 10):
        #rospy.init_node("tablegrasps_dataset_generator")

        self.num_modposes = num_model_poses # random poses for table grasps.
        self.hand_name = hand_name
        self.models_path = models_path
        self.models_subdir = None
        self.pregrasp_params = hand_pregrasp_params

        # Load dataset from specified dir.
        self.dset = GraspDataset(dset_filepath, dset_config_filepath, 
                                 dset_must_exist = True)
        rospy.loginfo("Successfully loaded dataset.")
        self.dset.set_tgrasp_mode() # TODO hacky way to reset index counters.
        self.dset_iter = self.dset.iterator(
            start = self.dset.get_current_model_index()) 
        rospy.on_shutdown(self.dset.close_dataset)
        rospy.loginfo("Dataset location:\t%s" % dset_filepath)
        rospy.loginfo("Dataset config location:\t%s" % dset_config_filepath)
        if self.dset.get_current_model_name() != "":
            rospy.loginfo("Current model name:\t%s" % 
                          self.dset.get_current_model_name())

        # Get relative path for loading models:
        if models_path is not None:
            # Graspit loads models relative to $GRASPIT/models/objects:
            self.models_subdir = models_path.split("models/objects/")[1]
            self.dset.set_metadata("models_subdir",self.models_subdir)
        else:
            # If no model path given, must already be in dset metadata: 
            rospy.loginfo("models_path is None.")
            if self.dset.get_metadata("models_subdir") is None:
                rospy.logfatal("No model subdirectory found in dataset.\
                             Unable to load models from names in dataset.")
            else:
                self.models_subdir = self.dset.get_metadata("models_subdir")

        rospy.loginfo("Finished setting up.")

    # Helper for genTGraspNextMod and genTGraspAllMod.s:
    def generateTableGraspOneModel(self, data):
        rospy.wait_for_service("findTableGrasps")
        try: 
            model_name = data.model_name
            grasps = graspsFromArray(data.initial_grasp, data.dof_value)

            rospy.loginfo("\tGenerating table grasps for %s" % model_name)
            if self.models_subdir is not None:
                model_name = self.models_subdir + "/" + model_name
            ret = gc.findTableGrasps(grasps, self.hand_name, self.pregrasp_params, 
                                     model_name, self.num_modposes)

        except GraspitTimeoutException:
            rospy.logerr("Graspit commander timed out.  No grasps planned.")
            rospy.logerr("Please ensure that graspit_interface is running.")
        else:
            rospy.loginfo("Storing grasps in dataset.")
            self.dset.add_table_grasps(ret.hand_poses, ret.body_poses, 
                                       ret.table_poses, is_new_model=True)

    def generateTableGraspNextModel(self):
        # Get next model's data from iterator. Convert to Grasp msg:
        data = self.dset_iter.next()
        self.generateTableGraspOneModel(data)

    def generateTableGraspAllModels(self):
        for model in self.dset_iter:
            self.generateTableGraspOneModel(model)


def graspsFromArray(grasps_array, dofs):
    print "Creating Grasp object from array."
    print grasps_array
    print grasps_array.shape
    print dofs
    grasps = []
    for i in xrange(len(grasps_array)):
        g = Grasp()
        if hasattr(g.dofs,'__iter__'):
            g.dofs = dofs[i]
        else:
            g.dofs = dofs
        p, q = g.pose.position, g.pose.orientation
        q.w, q.x, q.y, q.z, p.x, p.y, p.z = tuple(grasps_array[i])
        grasps.append(g)
    return grasps

if __name__ == "__main__":
    # Set up a dataset and generate initial_grasps for it:
    dset_filepath = "/home/jack/ros_stuff/my_ws/src/grasp_dataset_generation/test/test.h5"
    config_filepath = "/home/jack/ros_stuff/my_ws/src/grasp_dataset_generation/dataset_configs/just_grasps.yaml"
    models_path = "/home/jack/ros_stuff/my_ws/src/graspit-ros/graspit/graspit_source/models/objects/bigbird_subset"
    hand_name = "baxter"

    rospy.init_node("initial_dataset_generator")

    dset_builder = InitialGraspDsetBuilder(dset_filepath, config_filepath, hand_name, models_path = models_path)

    dset_builder.generateGraspAllModels()

    
