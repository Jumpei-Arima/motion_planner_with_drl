import tensorflow as tf
import numpy as np

class PPONet(object):
    def __init__(self,sess,NUM_STATES,NUM_ACTIONS,A_BOUNDS,NUM_HIDDENS):
        self.sess = sess
        self.num_states = NUM_STATES
        self.num_actions = NUM_ACTIONS
        self.a_bounds = A_BOUNDS
        self.num_hiddens = NUM_HIDDENS

        self.s_t = tf.placeholder(tf.float32, shape=(None, self.num_states))
        self.a_t = tf.placeholder(tf.float32, shape=(None, self.num_actions))

        self.alpha, self.beta, self.v, self.params = self._build_net('pi',trainable=True)
        self.graph = self.build_graph()

    def _build_net(self,name,trainable):
        with tf.variable_scope(name):
            l1 = tf.layers.dense(self.s_t, self.num_hiddens[0],tf.nn.relu,trainable=trainable)
            l2 = tf.layers.dense(l1, self.num_hiddens[1],tf.nn.relu,trainable=trainable)
            l3 = tf.layers.dense(l2, self.num_hiddens[2],tf.nn.relu,trainable=trainable)
            alpha = tf.layers.dense(l3,self.num_actions,tf.nn.softplus,trainable=trainable)
            beta = tf.layers.dense(l3,self.num_actions,tf.nn.softplus,trainable=trainable)
            value = tf.layers.dense(l3,1,trainable=trainable)
        params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES,scope=name)

        return alpha, beta, value, params

    def build_graph(self):
        x = (self.a_t - self.a_bounds[0]) / (-self.a_bounds[0] + self.a_bounds[1])
        beta_dist = tf.contrib.distributions.Beta(self.alpha + 1, self.beta + 1)
        self.prob = beta_dist.prob( x ) + 1e-8

        self.A = beta_dist.sample(1) * (-self.a_bounds[0] + self.a_bounds[1]) + self.a_bounds[0]

    def predict_a(self, s):
        a = self.sess.run(self.A, feed_dict={self.s_t: s})
        return a
