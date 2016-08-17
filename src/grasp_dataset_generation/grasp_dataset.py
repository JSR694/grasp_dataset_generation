import h5py
import yaml
from collections import namedtuple
from os.path import isfile
import random
import numpy as np
from graspit_interface.msg import Grasp
from graspit_interface.srv import FindTableGrasps

class GraspDataset():

    def __init__(self, dset_full_filepath, dset_config_full_filepath=None, 
                 dset_must_exist = False):
        self.dset_full_filepath = dset_full_filepath
        self.dset_config_full_filepath = dset_config_full_filepath

        if dset_must_exist and not isfile(dset_full_filepath):
            print "ERROR: Dataset file does not exist:"
            print "\t%s" % dset_full_filepath
            raise Exception("Dataset file does not exist.")

        self.config = yaml.load(open(dset_config_full_filepath))
        self.dset = h5py.File(self.dset_full_filepath,mode='a')

        # Iterate through config file. Create any dsets not already in the dbase:
        for dset_key in self.config.keys():
            if not dset_key in self.dset.keys():
                table_config = self.config[dset_key]
                # Create special type dset (variable length string):
                if 'dtype' in table_config and table_config['dtype'] == 'String':
                    dt = h5py.special_dtype(vlen=bytes)
                    self.dset.create_dataset(name=dset_key,
                                             shape=tuple(table_config['chunk_shape']),
                                             maxshape=tuple(table_config['max_shape']),
                                             chunks=tuple(table_config['chunk_shape']),
                                             dtype=dt)
                # Create standard type dset:
                else:
                    self.dset.create_dataset(name=dset_key,
                                             shape=tuple(table_config['chunk_shape']),
                                             maxshape=tuple(table_config['max_shape']),
                                             chunks=tuple(table_config['chunk_shape']))
                # Add any attributes:
                if "attrs" in table_config.keys():
                    for attr,val in table_config['attrs']:
                        self.dset[dset_key].attrs[attr] = val

        #This is the index of the next grasp location, in initial_grasps xor
        # grasps_on_table.
        if not 'current_grasp_index' in self.dset.keys():
            self.dset.create_dataset(name='current_grasp_index', shape=(1,))
            self.dset['current_grasp_index'][0] = 0

        #index of the next model name location:
        if not 'current_model_index' in self.dset.keys():
            self.dset.create_dataset(name='current_model_index', shape=(1,))
            self.dset['current_model_index'][0] = 0

        #index of the next model name location (FOR table_grasps):
        if not 'current_model_pose_index' in self.dset.keys():
            self.dset.create_dataset(name='current_model_pose_index', shape=(1,))
            self.dset['current_model_pose_index'][0] = 0

        # Define named tuples for data relevant to initial/table grasps:
        ig_keys, tg_keys = [], []
        for key in self.config.keys():
            if "initial_grasps" in self.dset[key].attrs.keys():
                ig_keys.append(key)
            if "table_grasps" in self.dset[key].attrs.keys():
                tg_keys.append(key)
        self.InitGraspData = namedtuple('InitGrasp', ig_keys)
        self.TableGraspData = namedtuple('TableGrasp', tg_keys)

    def close_dataset(self):
        self.dset.close()
        print "Safely closed dataset."

    def get_current_model_name(self):
        i = self.dset['current_model_index'][0]
        return self.dset['model_name'][i]
    
    # An additional dataset that can store useful metadata as its attributes.
    #  Example: storing directory information while building the dataset.
    def set_metadata(self, name, val):
        if not "metadata" in self.dset.keys():
            self.dset.create_dataset(name="metadata", shape=(1,))
        self.dset["metadata"].attrs[name] = val

    def get_metadata(self, name):
        if not "metadata" in self.dset.keys():
            print "ERROR! No metadata in this dataset."
            return None
        elif not name in self.dset["metadata"].attrs.keys():
            print "WARNING! No metadata field with name %s." % name
            return None
        else:
            return self.dset["metadata"].attrs[name]


    def add_models(self, model_names):
        # Resize dsets as necessary:
        current_limit = self.dset["model_name"].shape[0]
        current_index = self.dset["model_name"].attrs["num_models"]
        if current_index-1 + len(model_names) > current_limit:
            difference = current_index-1 + len(model_names) - current_limit
            print "RESIZING DSET TO FIT %d MORE NAMES" % difference
            print "NEW SIZE (on axis 0):\t%d+%d=  %d", (current_limit, difference, current_limit+difference)
            # All datasets indexed by model on first axis.  Resize:
            for key in self.config.keys():
                self.dset[key].resize(current_limit + difference +1, axis=0) #TODO the +1 is a kludge. Actually fix off-by-one-error!
        

        self.dset["model_name"][current_index:current_index+len(model_names)] = model_names
        current_index += len(model_names)
        self.dset["model_name"].attrs["num_models"] = current_index
        """
        # Add names:
        for name in model_names:
            print self.dset["model_name"][current_index]
            print type(self.dset["model_name"][current_index])
            print self.dset["model_name"][current_index]
            self.dset["model_name"][current_index] = [name]
            current_index +=1 
        self.dset["model_name"].attrs["num_models"] = current_index
        """

    #ef mark_current_index(self, is_table_grasps):
    #   if is_table_grasps:
    #       # Only get items relevant to table_grasps:
    #       has_attr = lambda k: "table_grasps" in self.dset[k].attrs.keys()
    #       for key in filter(has_attr, self.config.keys()):
    #           # indexed by model and modpose alone:
    #           if len(self.dset[key].shape) == 3:
    #               grasp_dict[key] = self.dset[key][model_index]
    #           # indexed by model, modpose, and grasp:
    #           elif self.dset(["current_grasp_index"]).shape[0]
    #               grasp_dict[key] = self.dset[key][model_index][model_pose_index][grasp_index]
    #   else:
    #   # Only get items relevant to initial_grasps:
    #       not_has_attr = lambda k: not ("table_grasps" in self.dset[k].attrs.keys())
    #       for key in filter(not_has_attr, self.config.keys()):
    #           # indexed by model name alone:
    #           if len(self.dset[key].shape) == 2:
    #               grasp_dict[key] = self.dset[key][model_index]
    #           # indexed by model and grasp:
    #           else:
    #           grasp_dict[key] = self.dset[key][model_index][grasp_index]

    #TODO is this all garbage?:
    #this method will do just one resize, rather
    #than doing so at every nth insertion.  This should
    #be much faster.
    # grasps is a list of Grasp messages.
    #ef add_table_grasps(self, grasps, table_grasp_indices=(None,None)):
    #   if table_grasp_indices != (None,None):
    #       model_index, model_pose_index = table_grasp_indices
    #       if (model_index >= self.dset(["current_model_index"]).shape[0] or 
    #               model_pose_index >= self.dset(["current_model_pose_index"]).shape[0]):
    #           print "ERROR!  One or both indices out of bounds:   \
    #               Model: %d\tModel pose: %d" % (model_index, model_pose_index)
    #           return None
    #       else:
    #           self.dset["current_model_index"][0] = model_index
    #           self.dset["current_model_pose_index"][0] = model_pose_index
    #   current_limit = self.dset[self.config.keys()[0]].shape[-1]
    #   current_index = self.get_current_index()
    #   if current_index + len(grasps) > current_limit:
    #       difference = current_index + len(grasps) - current_limit
    #       has_attr = lambda k: "table_grasps" in self.dset[k].attrs.keys()
    #       for key in filter(has_attr, self.config.keys()):
    #           num_axes = len(self.dset["grasps_on_table"].shape)
    #           self.dset[key].resize(current_limit + difference, axis=num_axes)

    #   # Add grasps:
    #   for grasp in grasps:
    #



    # Add multiple grasps for a single model.
    # By default, adds grasps to the model specified by "current_model_index".
    #   If is_new_model==True, increments current_model_index to move on to subsequent model.
    def add_initial_grasps(self, grasps, energies, is_new_model=False):
        if is_new_model:
            # Don't skip index 0:
            if not self.get_metadata("grasps_added_for_model_zero"):
                self.set_metadata("grasps_added_for_model_zero", True)
            # Check that there is a next model:
            elif self.dset["current_model_index"][0] == self.dset["model_name"].shape[0]:
                print "ERROR! Cannot move to next model-- reached end of current model list."
                return
            else:
                self.dset["current_model_index"][0]+=1
                self.dset["current_grasp_index"][0] = 0

        # Resize datasets if necessary:
        current_limit = self.dset["current_grasp_index"].shape[0]
        current_index = self.dset["current_grasp_index"][0]
        if current_index + len(grasps) > current_limit:
            difference = current_index + len(grasps) - current_limit
            # Resize all datasets which have an axis for grasps:
            has_attr = lambda k: "grasps_axis" in self.dset[k].attrs.keys()
            for key in filter(has_attr, self.config.keys()):
                grasps_axis = self.dset[key].attrs["grasps_axis"]
                self.dset[key].resize(current_limit + difference, axis=grasps_axis)

        # Add grasps:
        model_index = self.dset["current_model_index"][0]
        for i in xrange(len(grasps)):
            current_index = self.dset["current_grasp_index"][0]
            grasp, energy = grasps[i], energies[i]
            # Convert Grasp msg to dataset entries:
            p,q = grasp.pose.position, grasp.pose.orientation
            grasp_arry = np.array([q.w, q.x, q.y, q.z, p.x, p.y, p.z])
            #energy_arry = np.array([grasp.epsilon_quality, grasp.volume_quality])
            print "GRASP ",i
            print "model index, current_index: ",model_index,current_index
            print "state of dataset fields before writing:"
            print self.dset["initial_grasp"][model_index][current_index]
            print self.dset["dof_value"][model_index][current_index][0] 
            print "values written:"
            print grasp_arry
            print grasp.dofs[0]
            self.dset["initial_grasp"][model_index,current_index] = grasp_arry
            self.dset["dof_value"][model_index,current_index,0] = grasp.dofs[0]
            #self.dset["energy"][model_index][current_index] = energy_arry
            # TODO this is an assumption about the hand-- that is has one DOF:
            self.dset["energy"][model_index,current_index,0] = energy

            self.dset["current_grasp_index"][0]+=1

    # TODO super hacky way to reset c_g_idx after adding init_grasps:
    def set_tgrasp_mode(self):
        self.dset["current_grasp_index"][0]=0
    #TODO test add_table_grasps!

    # Add multiple table grasps for a single model.
    #  Table grasps are initial_grasps, transformed to the object's frame after the object is placed
    #  on a table in a random pose.
    # By default, adds grasps to the model specified by "current_model_index".
    #   If is_new_model==True, increments current_model_index to move on to subsequent model.
    def add_table_grasps(self, tgrasps, obj_poses, tposes, 
                         is_new_model=False):
        print "state at call:"
        print       self.dset["current_model_index"][0]
        print       self.dset["current_grasp_index"][0]
        print       self.dset["current_model_pose_index"][0]
        if is_new_model:
            if not self.get_metadata("table_grasps_added_for_model_zero"):
                self.set_metadata("table_grasps_added_for_model_zero", True)
            # Check that there is a next model:
            elif self.dset["current_model_index"][0] == self.dset["model_name"].shape[0]:
                print "ERROR! Cannot move to next model-- reached end of current model list."
                return  #TODO raise an actual exception?
            else:
                # TODO there must be off-by-one err with cur_mod_ix here:
                self.dset["current_model_index"][0]+=1
                print "incremented cur_mod_idx to %d" % self.dset["current_model_index"][0]
                self.dset["current_grasp_index"][0] = 0
                self.dset["current_model_pose_index"][0] = 0

        # Asssuming same # grasps for each model.
        num_grasps = len(tgrasps[0].tgp) # grasp list for 1st model pose.
        num_modposes = len(tgrasps)

        # Resize datasets if necessary, on model_pose and grasp axes:
        current_limit = self.dset["current_grasp_index"].shape[0]
        current_index = self.dset["current_grasp_index"][0]
        if current_index + num_grasps > current_limit:
            difference = current_index + num_grasps - current_limit
            # Resize all datasets which have an axis for grasps:
            has_attr = lambda k: "grasps_axis" in self.dset[k].attrs.keys()
            for key in filter(has_attr, self.config.keys()):
                grasps_axis = self.dset[key].attrs["grasps_axis"]
                self.dset[key].resize(current_limit + difference, axis=grasps_axis)

        current_limit = self.dset["current_model_pose_index"].shape[0]
        current_index = self.dset["current_model_pose_index"][0]
        if current_index + num_modposes > current_limit:
            difference = current_index + num_modposes - current_limit
            # Resize all datasets which have an axis for model_pose:
            has_attr = lambda k: "model_pose_axis" in self.dset[k].attrs.keys()
            for key in filter(has_attr, self.config.keys()):
                modpose_axis = self.dset[key].attrs["model_pose_axis"]
                self.dset[key].resize(current_limit + difference, axis=modpose_axis)

        # Add table and model poses:
        idx = self.dset["current_model_pose_index"][0]
        for i in xrange(idx,num_modposes):
            self.dset["table_pose"][idx+i] = self.array_from_pose_msg(tposes[i])
            self.dset["model_pose"][idx+i] = self.array_from_pose_msg(obj_poses[i])

        # Add grasps and grasp validity:
        mod_idx = self.dset["current_model_index"][0]
        for modpose in tgrasps:
            modpose_idx = self.dset["current_model_pose_index"][0]
            g_idx = self.dset["current_grasp_index"][0]
            print "FIRST\tmod, g, modp:\t%d\t%d\t%d" % (mod_idx, g_idx, modpose_idx)
            for grasp in modpose.tgp:
                pose_arry = self.array_from_pose_msg(grasp.pose)
                self.dset["grasp_on_table"][mod_idx,g_idx,modpose_idx] = pose_arry
                self.dset["is_valid_grasp"][mod_idx,g_idx,modpose_idx] = grasp.is_valid
                g_idx+=1
            print "LAST\tmod, g, modp:\t%d\t%d\t%d" % (mod_idx, g_idx, modpose_idx)
            self.dset["current_model_pose_index"][0] += 1
        self.dset["current_grasp_index"][0] += num_grasps
        # TODO check consistency of current_xxxx_index.  Should give first UNASSIGNED index.
        # TODO loginfo for this function.


    def array_from_pose_msg(self, pose):
            p, q = pose.position, pose.orientation
            return np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])

    # Get all table grasps for a given model:
    #TODO cleanup
    def get_table_grasp(self, model_index=None):
        grasp_dict = {}

        if model_index is None:
            model_index = self.dset["current_model_index"][0] 
        #if model_pose_index is None:
        #    model_pose_index = self.dset["current_model_pose_index"][0] 

        # Only get items relevant to table_grasps:
        has_attr = lambda k: "table_grasps" in self.dset[k].attrs.keys()
        for key in filter(has_attr, self.config.keys()):
            # indexed by model name alone:
            #if len(self.dset[key].shape) == 2:
            grasp_dict[key] = self.dset[key][model_index]
            # indexed by model and modpose:
            #elif len(self.dset[key].shape) == 3:
            #   grasp_dict[key] = self.dset[key][model_index]
            # indexed by model, modpose, and grasp:
            #else:
            #    grasp_dict[key] = self.dset[key][model_index][model_pose_index][grasp_index]

        return self.TableGraspData(**grasp_dict)

    # Get all initial_grasps for a given model.
    # TODO cleanup
    def get_initial_grasp(self, model_index=None):
        grasp_dict = {}

        if model_index is None:
            model_index = self.dset["current_model_index"][0] 

        # Only get items relevant to initial_grasps:
        has_attr = lambda k: "initial_grasps" in self.dset[k].attrs.keys()
        for key in filter(has_attr, self.config.keys()):
            # indexed by model name alone:
            #if len(self.dset[key].shape) == 2:
            grasp_dict[key] = self.dset[key][model_index]
            # indexed by model and grasp:
            #else:
            #    grasp_dict[key] = self.dset[key][model_index][grasp_index]

        return self.InitGraspData(**grasp_dict)

    def get_current_grasp_index(self):
        return self.dset['current_grasp_index'][0]

    # table_grasp_indices specifies which model and model_pose, for grasp_on_table:
    def iterator(self, start=None, end=None, table_grasp_indices=(None,None)):
        # If iterator of table_grasps, rather than initial_grasps:
        if table_grasp_indices != (None,None):
            model_index, model_pose_index = table_grasp_indices
            if (model_index >= self.dset(["current_model_index"]).shape[0] or 
                    model_pose_index >= self.dset(["current_model_pose_index"]).shape[0]):
                print "ERROR!  One or both indices out of bounds:   \
                    Model: %d\tModel pose: %d" % (model_index, model_pose_index)
                return None
            else:
                self.dset["current_model_index"][0] = model_index
                self.dset["current_model_pose_index"][0] = model_pose_index
            return GraspIterator(self, start, end, table_grasps=True)
        else: # Else iterator of initial grasps.
            return GraspIterator(self, start, end, table_grasps=False)

    # table_grasp_indices specifies which model and model_pose, for grasp_on_table:
    def random_iterator(self, num_items=None, table_grasp_indices=(None,None)):
        # If iterator of table_grasps, rather than initial_grasps:
        if table_grasp_indices != (None,None):
            model_index, model_pose_index = table_grasp_indices
            if (model_index >= self.dset(["current_model_index"]).shape[0] or 
                    model_pose_index >= self.dset(["current_model_pose_index"]).shape[0]):
                print "ERROR!  One or both indices out of bounds:   \
                    Model: %d\tModel pose: %d" % (model_index, model_pose_index)
                return None
            else:
                self.dset["current_model_index"][0] = model_index
                self.dset["current_model_pose_index"][0] = model_pose_index
            return RandomGraspIterator(self, num_items, table_grasps=True)
        else: # Else iterator of initial grasps.
            return RandomGraspIterator(self, start, end, table_grasps=False)


#GraspIterator can iterate over initial_grasps, or grasps_on_table.
# Returns all the initial_grasps, or table_grasps (and table, model pose)
#   for a model.
class GraspIterator():

    def __init__(self, dataset, start=None, end=None, table_grasps=False):
        self.table_grasps = table_grasps
        self.dataset = dataset
        self.current_index = 0

        if end is not None:
            self.end_index = end
        else:
            #TODO num_models should really be get_metadata attr:
            self.end_index = self.dataset.dset["model_name"].attrs["num_models"]

        if start is not None:
            self.current_index = start

    def __iter__(self):
        return self

    def next(self):

        #we have iterated over all the grasps
        if self.current_index >= self.end_index:
            raise StopIteration()

        if self.table_grasps:
            return self.dataset.get_table_grasp(self.current_index)
        else:
            return self.dataset.get_initial_grasp(self.current_index)
        self.current_index += 1

        return grasp


class RandomGraspIterator():
    # TODO update to iterate over models, not over grasps or whatever:
    def __init__(self, dataset, num_items=None, table_grasps=False):
        self.table_grasps = table_grasps
        self.dataset = dataset

        if num_items is None:
            num_items = self.dataset.get_current_grasp_index()

        self.order = np.random.permutation(int(self.dataset.dset["current_model_index"]))

        self.num_items = num_items

        self.current_index = 0

    def __iter__(self):
        return self

    def next(self):
        #we have iterated over all the grasps
        if self.current_index >= self.num_items:
            raise StopIteration()

        if self.table_grasps:
            return self.dataset.get_table_grasp(self.order[self.current_index])
        else:
            return self.dataset.get_initial_grasp(self.order[self.current_index])
    
        self.current_index += 1
