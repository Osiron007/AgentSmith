#Basics
import os
import time

#Tensorflow
import tensorflow as tf

#Keras
from keras import backend as K

#ANN-Model
from model import ActorNetwork


class ANN(object):

    def __init__(self, path_to_weights, weights_id):

        print("Init ANN")

        self.init = False

        # aviod TF from allocation all GPU mem
        # https://stackoverflow.com/questions/34199233/how-to-prevent-tensorflow-from-allocating-the-totality-of-a-gpu-memory
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        #sess = tf.Session()
        K.set_session(sess)

        self.path_to_weights = path_to_weights
        self.next_weights_id = weights_id

        #check if path is valid
        if not os.path.isdir(self.path_to_weights):
            print("Folder with weights does not exist!")
            print(self.path_to_weights)
            self.init = False
        else:
            self.init = True

        if self.init:
            #create actor network
            self.actor = ActorNetwork.ActorNetwork(sess, 5, 2)
            #load first weights
            print("Loading first weights for ANN from " + str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
            try:
                self.actor.model.load_weights(
                    str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
                self.next_weights_id = self.next_weights_id + 1
            except:
                print("Cannot find the weight (.h5) file")
                print(str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
                self.init = False

    def is_init(self):
        return self.init

    def load_new_weights(self):

        print("Search newest weights")
        search_high_number = True
        tmp_next_weights_id = self.next_weights_id
        new_weights_found = False

        while search_high_number:
            if os.path.exists(str(self.path_to_weights) + "/" + str(tmp_next_weights_id) + "/actormodel.h5"):
                #check if there is a newer one
                tmp_next_weights_id = tmp_next_weights_id + 1
                new_weights_found = True
            else:
                #stop searching
                tmp_next_weights_id = tmp_next_weights_id - 1
                search_high_number = False

        if not new_weights_found:
            print("New weights NOT found")
            return False

        print("New weights found")
        self.next_weights_id = tmp_next_weights_id

        print("Waiting 5 sec before reading file")
        print(str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
        #wait 5sec to make sure write process is done
        time.sleep(5)

        print("Loading new weights for ANN")
        try:
            self.actor.model.load_weights(
                str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
            print("Weight load successfully")
            self.next_weights_ID = self.next_weights_id + 1
        except:
            print("Cannot find the weight (.h5) file")
            print(str(self.path_to_weights) + "/" + str(self.next_weights_id) + "/actormodel.h5")
            return False

        self.next_weights_id = self.next_weights_id + 1
        return True

    def generate_action(self, state):
        # generate action with ANN

        print("gernerating action")

        action = self.actor.model.predict(state)

        return action
