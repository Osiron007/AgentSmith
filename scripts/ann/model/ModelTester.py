from ActorNetwork import ActorNetwork
import tensorflow as tf

from keras.utils import plot_model

sess = tf.Session()
from keras import backend as K

K.set_session(sess)

state_dim = 5

action_dim = 2

BATCH_SIZE = 32

TAU = 0.01

LRA = 0.001

actor = ActorNetwork(sess, state_dim, action_dim)

print(actor.model.summary())

actor.model.save_weights("actormodel.h5", overwrite=True)

#print(actor.target_model.summary())

#plot_model(actor.model, show_shapes=True, to_file='ActorModel_Exp8-5.png')
#plot_model(critic.model, show_shapes=True, to_file='CriticModel_Exp8-5.png')

