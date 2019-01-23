from keras.models import Model
from keras.layers import Dense, Flatten, Input, merge, Lambda
from keras.layers.merge import concatenate
from keras.initializers import normal, RandomNormal
import tensorflow as tf
import keras.backend as K


class ActorNetwork(object):
    def __init__(self, sess, state_size, action_size):

        self.sess = sess
        K.set_session(sess)

        #Now create the model
        self.model, self.weights, self.state = self.create_actor_network(state_size, action_size)
        self.sess.run(tf.global_variables_initializer())


    def create_actor_network(self, state_size, action_dim):
        print("Now we build the model ActorV2")
        S = Input(shape=[state_size], name='InputLayer', dtype='float32')
        h0 = Dense(1024, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer0')(S)
        h1 = Dense(512, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer1')(h0)
        h2 = Dense(512, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer2')(h1)

        # to create a final layer with maybe different activation functions
        # create a layer for each activation function
        # tanh is in range of -1 to 1

        omega_left_wheel = Dense(1, activation='tanh', kernel_initializer=RandomNormal(stddev=1e-4), name='action_left_wheel')(h2)
        omega_right_wheel = Dense(1, activation='tanh', kernel_initializer=RandomNormal(stddev=1e-4), name='action_right_wheel')(h2)

        # and merge them later
        #V = merge([omega_left_wheel, omega_right_wheel], mode='concat', name='OutputLayer')
        V = concatenate([omega_left_wheel, omega_right_wheel], name='OutputLayer')

        # define the model from layers
        model = Model(inputs=S, outputs=V, name='ActorModel')

        return model, model.trainable_weights, S

